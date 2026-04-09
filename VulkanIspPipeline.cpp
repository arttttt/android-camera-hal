#define LOG_TAG "Cam-VulkanISP"
#include <utils/Log.h>
#include <cstring>
#include <unistd.h>
#include <linux/videodev2.h>
#include <math.h>
#include <time.h>

static inline int64_t nowMs() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000LL + ts.tv_nsec / 1000000;
}

#include "VulkanIspPipeline.h"
#include "bayer_isp_spv.h"

/* Missing KHR types in old Vulkan SDK (android-24) */
#ifndef VK_STRUCTURE_TYPE_IMPORT_MEMORY_FD_INFO_KHR
#define VK_STRUCTURE_TYPE_IMPORT_MEMORY_FD_INFO_KHR 1000074000
#define VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT_KHR 0x00000001
typedef struct VkImportMemoryFdInfoKHR {
    VkStructureType sType;
    const void *pNext;
    uint32_t handleType;  /* VkExternalMemoryHandleTypeFlagBitsKHR */
    int fd;
} VkImportMemoryFdInfoKHR;
#endif

namespace android {

uint8_t VulkanIspPipeline::sGammaLut[256];
bool VulkanIspPipeline::sGammaReady = false;

void VulkanIspPipeline::initGamma() {
    if (sGammaReady) return;
    for (int i = 0; i < 256; i++) {
        float lin = i / 255.0f;
        float s = (lin <= 0.0031308f) ?
            lin * 12.92f :
            1.055f * powf(lin, 1.0f / 2.4f) - 0.055f;
        int v = (int)(s * 255.0f + 0.5f);
        sGammaLut[i] = v > 255 ? 255 : (v < 0 ? 0 : (uint8_t)v);
    }
    sGammaReady = true;
}

VulkanIspPipeline::VulkanIspPipeline()
    : mReady(false), mDmabufSupported(false), mBufWidth(0), mBufHeight(0)
    , mInstance(VK_NULL_HANDLE), mPhysDev(VK_NULL_HANDLE)
    , mDevice(VK_NULL_HANDLE), mQueue(VK_NULL_HANDLE)
    , mShader(VK_NULL_HANDLE), mDescLayout(VK_NULL_HANDLE)
    , mPipeLayout(VK_NULL_HANDLE), mPipeline(VK_NULL_HANDLE)
    , mDescPool(VK_NULL_HANDLE), mDescSet(VK_NULL_HANDLE)
    , mCmdPool(VK_NULL_HANDLE), mCmdBuf(VK_NULL_HANDLE)
    , mInBuf(VK_NULL_HANDLE), mOutBuf(VK_NULL_HANDLE), mParamBuf(VK_NULL_HANDLE)
    , mInMem(VK_NULL_HANDLE), mOutMem(VK_NULL_HANDLE), mParamMem(VK_NULL_HANDLE)
    , mInSize(0), mOutSize(0)
    , mInMap(NULL), mOutMap(NULL), mParamMap(NULL)
    , mDmaBuf(VK_NULL_HANDLE), mDmaMem(VK_NULL_HANDLE), mDmaFd(-1) {}

VulkanIspPipeline::~VulkanIspPipeline() { destroy(); }

/* --- helpers --- */

void VulkanIspPipeline::fillParams(IspParams *p, unsigned w, unsigned h,
                                    bool is16, uint32_t pixFmt) {
    memset(p, 0, sizeof(IspParams));
    p->width = w; p->height = h;
    p->is16bit = is16 ? 1 : 0;
    p->doIsp = mEnabled ? 1 : 0;
    p->wbR = mWbR; p->wbG = mWbG; p->wbB = mWbB;

    switch (pixFmt) {
        case V4L2_PIX_FMT_SRGGB10: case V4L2_PIX_FMT_SRGGB8: p->bayerPhase = 0; break;
        case V4L2_PIX_FMT_SGRBG10: case V4L2_PIX_FMT_SGRBG8: p->bayerPhase = 1; break;
        case V4L2_PIX_FMT_SGBRG10: case V4L2_PIX_FMT_SGBRG8: p->bayerPhase = 2; break;
        case V4L2_PIX_FMT_SBGGR10: case V4L2_PIX_FMT_SBGGR8: p->bayerPhase = 3; break;
        default: p->bayerPhase = 0; break;
    }

    if (mCcm) {
        for (int i = 0; i < 9; i++) p->ccm[i] = mCcm[i];
    } else {
        p->ccm[0] = 1024; p->ccm[4] = 1024; p->ccm[8] = 1024;
    }

    for (int i = 0; i < 64; i++) {
        p->gammaLut[i] = sGammaLut[i*4] |
                         (sGammaLut[i*4+1] << 8) |
                         (sGammaLut[i*4+2] << 16) |
                         (sGammaLut[i*4+3] << 24);
    }
}

void VulkanIspPipeline::updateAwb(const uint8_t *raw, unsigned w, unsigned h,
                                    bool is16, uint32_t pixFmt) {
    if (!mEnabled || !raw) return;

    uint64_t sR = 0, sG = 0, sB = 0, nR = 0, nG = 0, nB = 0;
    unsigned rX = (pixFmt == V4L2_PIX_FMT_SGRBG10 || pixFmt == V4L2_PIX_FMT_SGRBG8 ||
                   pixFmt == V4L2_PIX_FMT_SBGGR10 || pixFmt == V4L2_PIX_FMT_SBGGR8) ? 1 : 0;
    unsigned rY = (pixFmt == V4L2_PIX_FMT_SGBRG10 || pixFmt == V4L2_PIX_FMT_SGBRG8 ||
                   pixFmt == V4L2_PIX_FMT_SBGGR10 || pixFmt == V4L2_PIX_FMT_SBGGR8) ? 1 : 0;

    for (unsigned y = 0; y < h; y += 7) {
        for (unsigned x = 0; x < w; x += 7) {
            unsigned val = is16 ? ((const uint16_t *)raw)[y * w + x] >> 2
                                : raw[y * w + x];
            unsigned px = x & 1, py = y & 1;
            if (py == rY && px == rX) { sR += val; nR++; }
            else if (py != rY && px != rX) { sB += val; nB++; }
            else { sG += val; nG++; }
        }
    }

    if (nR && nG && nB) {
        uint64_t avgR = sR / nR, avgG = sG / nG, avgB = sB / nB;
        uint64_t avg = (avgR + avgG + avgB) / 3;
        unsigned r = avgR ? (unsigned)((avg * 256ULL) / avgR) : 256;
        unsigned g = avgG ? (unsigned)((avg * 256ULL) / avgG) : 256;
        unsigned b = avgB ? (unsigned)((avg * 256ULL) / avgB) : 256;
        if (r < 128) r = 128; if (r > 1024) r = 1024;
        if (g < 128) g = 128; if (g > 1024) g = 1024;
        if (b < 128) b = 128; if (b > 1024) b = 1024;
        mWbR = r; mWbG = g; mWbB = b;
    }
}

/* --- Vulkan buffer management --- */

uint32_t VulkanIspPipeline::findMemoryType(uint32_t filter, VkMemoryPropertyFlags props) {
    VkPhysicalDeviceMemoryProperties memProps;
    vkGetPhysicalDeviceMemoryProperties(mPhysDev, &memProps);
    for (uint32_t i = 0; i < memProps.memoryTypeCount; i++) {
        if ((filter & (1 << i)) && (memProps.memoryTypes[i].propertyFlags & props) == props)
            return i;
    }
    return UINT32_MAX;
}

bool VulkanIspPipeline::createBuffer(VkBuffer *buf, VkDeviceMemory *mem,
                                      VkDeviceSize size, VkBufferUsageFlags usage) {
    VkBufferCreateInfo ci = {};
    ci.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    ci.size = size;
    ci.usage = usage;
    ci.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    if (vkCreateBuffer(mDevice, &ci, NULL, buf) != VK_SUCCESS)
        return false;

    VkMemoryRequirements req;
    vkGetBufferMemoryRequirements(mDevice, *buf, &req);

    VkMemoryAllocateInfo ai = {};
    ai.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    ai.allocationSize = req.size;
    /* Prefer cached for fast CPU memcpy, fallback to coherent */
    ai.memoryTypeIndex = findMemoryType(req.memoryTypeBits,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_CACHED_BIT);
    if (ai.memoryTypeIndex == UINT32_MAX)
        ai.memoryTypeIndex = findMemoryType(req.memoryTypeBits,
            VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);

    if (ai.memoryTypeIndex == UINT32_MAX ||
        vkAllocateMemory(mDevice, &ai, NULL, mem) != VK_SUCCESS) {
        vkDestroyBuffer(mDevice, *buf, NULL);
        *buf = VK_NULL_HANDLE;
        return false;
    }

    vkBindBufferMemory(mDevice, *buf, *mem, 0);
    return true;
}

void VulkanIspPipeline::destroyBuffer(VkBuffer buf, VkDeviceMemory mem) {
    if (buf != VK_NULL_HANDLE) vkDestroyBuffer(mDevice, buf, NULL);
    if (mem != VK_NULL_HANDLE) vkFreeMemory(mDevice, mem, NULL);
}

bool VulkanIspPipeline::importDmabuf(int fd, VkDeviceSize size,
                                      VkBuffer *buf, VkDeviceMemory *mem) {
    VkBufferCreateInfo bci = {};
    bci.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    bci.size = size;
    bci.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
    bci.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    if (vkCreateBuffer(mDevice, &bci, NULL, buf) != VK_SUCCESS)
        return false;

    VkMemoryRequirements req;
    vkGetBufferMemoryRequirements(mDevice, *buf, &req);

    /* Import fd — OPAQUE_FD on Tegra goes through nvmap, same as V4L2 dmabuf */
    VkImportMemoryFdInfoKHR importInfo = {};
    importInfo.sType = (VkStructureType)VK_STRUCTURE_TYPE_IMPORT_MEMORY_FD_INFO_KHR;
    importInfo.handleType = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT_KHR;
    importInfo.fd = dup(fd);  /* Vulkan takes ownership */

    VkMemoryAllocateInfo ai = {};
    ai.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    ai.pNext = &importInfo;
    ai.allocationSize = req.size;
    ai.memoryTypeIndex = findMemoryType(req.memoryTypeBits,
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    if (ai.memoryTypeIndex == UINT32_MAX)
        ai.memoryTypeIndex = findMemoryType(req.memoryTypeBits, 0);

    if (vkAllocateMemory(mDevice, &ai, NULL, mem) != VK_SUCCESS) {
        ALOGW("importDmabuf: vkAllocateMemory failed");
        close(importInfo.fd);
        vkDestroyBuffer(mDevice, *buf, NULL);
        *buf = VK_NULL_HANDLE;
        return false;
    }

    vkBindBufferMemory(mDevice, *buf, *mem, 0);
    ALOGD("importDmabuf: fd=%d size=%zu OK", fd, (size_t)size);
    return true;
}

/* --- init / destroy --- */

bool VulkanIspPipeline::init() {
    if (mReady) return true;

    /* Instance */
    VkApplicationInfo appInfo = {};
    appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
    appInfo.pApplicationName = "CameraISP";
    appInfo.apiVersion = VK_API_VERSION;

    VkInstanceCreateInfo ici = {};
    ici.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
    ici.pApplicationInfo = &appInfo;

    if (vkCreateInstance(&ici, NULL, &mInstance) != VK_SUCCESS) {
        ALOGE("vkCreateInstance failed");
        return false;
    }

    /* Physical device */
    uint32_t devCount = 1;
    if (vkEnumeratePhysicalDevices(mInstance, &devCount, &mPhysDev) != VK_SUCCESS || devCount == 0) {
        ALOGE("No Vulkan physical device");
        destroy();
        return false;
    }

    /* Enumerate extensions — check for external memory support */
    bool hasExtMem = false, hasExtMemFd = false;
    {
        uint32_t extCount = 0;
        vkEnumerateDeviceExtensionProperties(mPhysDev, NULL, &extCount, NULL);
        VkExtensionProperties *exts = new VkExtensionProperties[extCount];
        vkEnumerateDeviceExtensionProperties(mPhysDev, NULL, &extCount, exts);
        for (uint32_t i = 0; i < extCount; i++) {
            ALOGD("VK ext: %s", exts[i].extensionName);
            if (!strcmp(exts[i].extensionName, "VK_KHR_external_memory"))
                hasExtMem = true;
            if (!strcmp(exts[i].extensionName, "VK_KHR_external_memory_fd"))
                hasExtMemFd = true;
        }
        delete[] exts;
    }
    mDmabufSupported = false; /* no instance ext support on K1 */

    /* Find compute queue */
    uint32_t qfCount = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(mPhysDev, &qfCount, NULL);
    VkQueueFamilyProperties qfProps[8];
    if (qfCount > 8) qfCount = 8;
    vkGetPhysicalDeviceQueueFamilyProperties(mPhysDev, &qfCount, qfProps);

    mQueueFamily = UINT32_MAX;
    for (uint32_t i = 0; i < qfCount; i++) {
        if (qfProps[i].queueFlags & VK_QUEUE_COMPUTE_BIT) {
            mQueueFamily = i;
            break;
        }
    }
    if (mQueueFamily == UINT32_MAX) {
        ALOGE("No compute queue");
        destroy();
        return false;
    }

    /* Logical device — enable external memory extensions if available */
    float qPriority = 1.0f;
    VkDeviceQueueCreateInfo qci = {};
    qci.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
    qci.queueFamilyIndex = mQueueFamily;
    qci.queueCount = 1;
    qci.pQueuePriorities = &qPriority;

    const char *extNames[] = {
        "VK_KHR_external_memory",
        "VK_KHR_external_memory_fd"
    };

    VkDeviceCreateInfo dci = {};
    dci.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
    dci.queueCreateInfoCount = 1;
    dci.pQueueCreateInfos = &qci;
    if (mDmabufSupported) {
        dci.enabledExtensionCount = 2;
        dci.ppEnabledExtensionNames = extNames;
    }

    if (vkCreateDevice(mPhysDev, &dci, NULL, &mDevice) != VK_SUCCESS) {
        ALOGE("vkCreateDevice failed (trying without extensions)");
        /* Fallback: create without extensions */
        mDmabufSupported = false;
        dci.enabledExtensionCount = 0;
        dci.ppEnabledExtensionNames = NULL;
        if (vkCreateDevice(mPhysDev, &dci, NULL, &mDevice) != VK_SUCCESS) {
            ALOGE("vkCreateDevice failed");
            destroy();
            return false;
        }
    }
    vkGetDeviceQueue(mDevice, mQueueFamily, 0, &mQueue);
    ALOGD("DMA-BUF import: %s", mDmabufSupported ? "enabled" : "disabled (memcpy path)");

    /* Shader module */
    ALOGD("INIT: creating shader module (size=%zu)", (size_t)bayer_isp_spv_len);
    VkShaderModuleCreateInfo smi = {};
    smi.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    smi.codeSize = bayer_isp_spv_len;
    smi.pCode = (const uint32_t *)bayer_isp_spv;

    if (vkCreateShaderModule(mDevice, &smi, NULL, &mShader) != VK_SUCCESS) {
        ALOGE("vkCreateShaderModule failed");
        destroy();
        return false;
    }
    ALOGD("INIT: shader module OK");

    /* Descriptor set layout: 3 storage buffers (input, output, params) */
    VkDescriptorSetLayoutBinding bindings[3] = {};
    bindings[0].binding = 0;
    bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    bindings[0].descriptorCount = 1;
    bindings[0].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    bindings[1] = bindings[0]; bindings[1].binding = 1;
    bindings[2] = bindings[0]; bindings[2].binding = 2;

    VkDescriptorSetLayoutCreateInfo dslci = {};
    dslci.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    dslci.bindingCount = 3;
    dslci.pBindings = bindings;
    ALOGD("INIT: creating descriptor set layout");
    vkCreateDescriptorSetLayout(mDevice, &dslci, NULL, &mDescLayout);
    ALOGD("INIT: descriptor set layout OK");

    /* Pipeline layout */
    VkPipelineLayoutCreateInfo plci = {};
    plci.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    plci.setLayoutCount = 1;
    plci.pSetLayouts = &mDescLayout;
    ALOGD("INIT: creating pipeline layout");
    vkCreatePipelineLayout(mDevice, &plci, NULL, &mPipeLayout);
    ALOGD("INIT: pipeline layout OK");

    /* Compute pipeline */
    VkComputePipelineCreateInfo cpci = {};
    cpci.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
    cpci.stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    cpci.stage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
    cpci.stage.module = mShader;
    cpci.stage.pName = "main";
    cpci.layout = mPipeLayout;

    ALOGD("INIT: creating compute pipeline");
    if (vkCreateComputePipelines(mDevice, VK_NULL_HANDLE, 1, &cpci, NULL, &mPipeline) != VK_SUCCESS) {
        ALOGE("vkCreateComputePipelines failed");
        destroy();
        return false;
    }

    /* Descriptor pool */
    VkDescriptorPoolSize poolSize = {};
    poolSize.type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    poolSize.descriptorCount = 3;

    VkDescriptorPoolCreateInfo dpci = {};
    dpci.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    dpci.maxSets = 1;
    dpci.poolSizeCount = 1;
    dpci.pPoolSizes = &poolSize;
    ALOGD("INIT: creating descriptor pool");
    vkCreateDescriptorPool(mDevice, &dpci, NULL, &mDescPool);
    ALOGD("INIT: descriptor pool OK");

    /* Allocate descriptor set */
    VkDescriptorSetAllocateInfo dsai = {};
    dsai.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    dsai.descriptorPool = mDescPool;
    dsai.descriptorSetCount = 1;
    dsai.pSetLayouts = &mDescLayout;
    ALOGD("INIT: allocating descriptor set");
    vkAllocateDescriptorSets(mDevice, &dsai, &mDescSet);
    ALOGD("INIT: descriptor set OK");

    /* Command pool + buffer */
    VkCommandPoolCreateInfo cpi = {};
    cpi.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    cpi.queueFamilyIndex = mQueueFamily;
    cpi.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    vkCreateCommandPool(mDevice, &cpi, NULL, &mCmdPool);

    VkCommandBufferAllocateInfo cbai = {};
    cbai.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    cbai.commandPool = mCmdPool;
    cbai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    cbai.commandBufferCount = 1;
    ALOGD("INIT: allocating command buffer");
    vkAllocateCommandBuffers(mDevice, &cbai, &mCmdBuf);
    ALOGD("INIT: command buffer OK");

    /* Params buffer — persistent map */
    if (!createBuffer(&mParamBuf, &mParamMem, sizeof(IspParams),
                      VK_BUFFER_USAGE_STORAGE_BUFFER_BIT)) {
        ALOGE("Failed to create params buffer");
        destroy();
        return false;
    }
    ALOGD("INIT: mapping params buffer");
    vkMapMemory(mDevice, mParamMem, 0, sizeof(IspParams), 0, &mParamMap);
    ALOGD("INIT: params mapped OK");

    initGamma();

    mReady = true;
    ALOGD("Vulkan ISP initialized");
    return true;
}

/* --- per-frame buffer management --- */

bool VulkanIspPipeline::ensureBuffers(unsigned width, unsigned height, bool is16bit) {
    size_t inSize = width * height * (is16bit ? 2 : 1);
    size_t outSize = width * height * 4;

    bool recreate = (mInSize < inSize || mOutSize < outSize ||
                     mBufWidth != width || mBufHeight != height);

    if (recreate) {
        /* Unmap before destroy */
        if (mInMap) { vkUnmapMemory(mDevice, mInMem); mInMap = NULL; }
        if (mOutMap) { vkUnmapMemory(mDevice, mOutMem); mOutMap = NULL; }

        destroyBuffer(mInBuf, mInMem); mInBuf = VK_NULL_HANDLE; mInMem = VK_NULL_HANDLE;
        destroyBuffer(mOutBuf, mOutMem); mOutBuf = VK_NULL_HANDLE; mOutMem = VK_NULL_HANDLE;

        if (!createBuffer(&mInBuf, &mInMem, inSize, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT) ||
            !createBuffer(&mOutBuf, &mOutMem, outSize, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT)) {
            ALOGE("Failed to allocate Vulkan buffers %ux%u", width, height);
            return false;
        }

        /* Persistent map */
        vkMapMemory(mDevice, mInMem, 0, inSize, 0, &mInMap);
        vkMapMemory(mDevice, mOutMem, 0, outSize, 0, &mOutMap);

        mInSize = inSize; mOutSize = outSize;
        mBufWidth = width; mBufHeight = height;
    }

    /* Always update descriptors — binding 0 may have been overridden by dmabuf path */
    VkDescriptorBufferInfo bufInfos[3] = {};
    bufInfos[0].buffer = mInBuf;  bufInfos[0].range = mInSize;
    bufInfos[1].buffer = mOutBuf; bufInfos[1].range = mOutSize;
    bufInfos[2].buffer = mParamBuf; bufInfos[2].range = sizeof(IspParams);

    VkWriteDescriptorSet writes[3] = {};
    for (int i = 0; i < 3; i++) {
        writes[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[i].dstSet = mDescSet;
        writes[i].dstBinding = i;
        writes[i].descriptorCount = 1;
        writes[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[i].pBufferInfo = &bufInfos[i];
    }
    vkUpdateDescriptorSets(mDevice, 3, writes, 0, NULL);

    return true;
}

/* --- main processing paths --- */

bool VulkanIspPipeline::process(const uint8_t *src, uint8_t *dst,
                                 unsigned width, unsigned height,
                                 uint32_t pixFmt) {
    if (!mReady) return false;

    bool is16 = (pixFmt == V4L2_PIX_FMT_SRGGB10 || pixFmt == V4L2_PIX_FMT_SGRBG10 ||
                 pixFmt == V4L2_PIX_FMT_SGBRG10 || pixFmt == V4L2_PIX_FMT_SBGGR10);

    if (!ensureBuffers(width, height, is16))
        return false;

    int64_t t0 = nowMs();

    /* Upload via persistent map */
    memcpy(mInMap, src, mInSize);

    /* Fill params via persistent map */
    IspParams params;
    fillParams(&params, width, height, is16, pixFmt);
    memcpy(mParamMap, &params, sizeof(IspParams));

    /* Flush CPU writes to device (no-op on coherent memory) */
    VkMappedMemoryRange flushRanges[2] = {};
    flushRanges[0].sType = VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE;
    flushRanges[0].memory = mInMem;
    flushRanges[0].size = VK_WHOLE_SIZE;
    flushRanges[1].sType = VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE;
    flushRanges[1].memory = mParamMem;
    flushRanges[1].size = VK_WHOLE_SIZE;
    vkFlushMappedMemoryRanges(mDevice, 2, flushRanges);

    int64_t t1 = nowMs();

    /* Record command buffer */
    VkCommandBufferBeginInfo beginInfo = {};
    beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

    vkResetCommandBuffer(mCmdBuf, 0);
    vkBeginCommandBuffer(mCmdBuf, &beginInfo);
    vkCmdBindPipeline(mCmdBuf, VK_PIPELINE_BIND_POINT_COMPUTE, mPipeline);
    vkCmdBindDescriptorSets(mCmdBuf, VK_PIPELINE_BIND_POINT_COMPUTE,
                            mPipeLayout, 0, 1, &mDescSet, 0, NULL);
    vkCmdDispatch(mCmdBuf, (width + 15) / 16, (height + 15) / 16, 1);
    vkEndCommandBuffer(mCmdBuf);

    /* Submit and wait */
    VkSubmitInfo si = {};
    si.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    si.commandBufferCount = 1;
    si.pCommandBuffers = &mCmdBuf;

    vkQueueSubmit(mQueue, 1, &si, VK_NULL_HANDLE);
    vkQueueWaitIdle(mQueue);
    int64_t t2 = nowMs();

    /* Invalidate output for CPU read (no-op on coherent memory) */
    VkMappedMemoryRange outRange = {};
    outRange.sType = VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE;
    outRange.memory = mOutMem;
    outRange.size = VK_WHOLE_SIZE;
    vkInvalidateMappedMemoryRanges(mDevice, 1, &outRange);

    /* Readback via persistent map */
    memcpy(dst, mOutMap, mOutSize);
    int64_t t3 = nowMs();

    ALOGD("VK: upload=%lld gpu=%lld readback=%lldms", t1 - t0, t2 - t1, t3 - t2);

    updateAwb(src, width, height, is16, pixFmt);
    return true;
}

bool VulkanIspPipeline::processFromDmabuf(int dmabufFd, const uint8_t *cpuFallback,
                                           uint8_t *dst, unsigned width, unsigned height,
                                           uint32_t pixFmt) {
    /* Fall through to memcpy path if dmabuf import not available */
    if (!mReady || !mDmabufSupported || dmabufFd < 0)
        return process(cpuFallback, dst, width, height, pixFmt);

    bool is16 = (pixFmt == V4L2_PIX_FMT_SRGGB10 || pixFmt == V4L2_PIX_FMT_SGRBG10 ||
                 pixFmt == V4L2_PIX_FMT_SGBRG10 || pixFmt == V4L2_PIX_FMT_SBGGR10);
    size_t inSize = width * height * (is16 ? 2 : 1);

    /* Ensure output buffer + all descriptors */
    if (!ensureBuffers(width, height, is16))
        return process(cpuFallback, dst, width, height, pixFmt);

    /* Import dmabuf as input — reimport if fd changed */
    if (mDmaFd != dmabufFd) {
        if (mDmaBuf != VK_NULL_HANDLE) destroyBuffer(mDmaBuf, mDmaMem);
        mDmaBuf = VK_NULL_HANDLE; mDmaMem = VK_NULL_HANDLE;
        if (!importDmabuf(dmabufFd, inSize, &mDmaBuf, &mDmaMem)) {
            ALOGW("dmabuf import failed, fallback to memcpy");
            mDmaFd = -1;
            return process(cpuFallback, dst, width, height, pixFmt);
        }
        mDmaFd = dmabufFd;
    }

    /* Override input descriptor to point to dmabuf */
    VkDescriptorBufferInfo bufInfo = {};
    bufInfo.buffer = mDmaBuf;
    bufInfo.range = inSize;
    VkWriteDescriptorSet w = {};
    w.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    w.dstSet = mDescSet;
    w.dstBinding = 0;
    w.descriptorCount = 1;
    w.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    w.pBufferInfo = &bufInfo;
    vkUpdateDescriptorSets(mDevice, 1, &w, 0, NULL);

    int64_t t0 = nowMs();

    /* Fill params via persistent map */
    IspParams params;
    fillParams(&params, width, height, is16, pixFmt);
    memcpy(mParamMap, &params, sizeof(IspParams));

    VkMappedMemoryRange paramRange = {};
    paramRange.sType = VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE;
    paramRange.memory = mParamMem;
    paramRange.size = VK_WHOLE_SIZE;
    vkFlushMappedMemoryRanges(mDevice, 1, &paramRange);

    int64_t t1 = nowMs();

    /* Dispatch */
    VkCommandBufferBeginInfo beginInfo = {};
    beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkResetCommandBuffer(mCmdBuf, 0);
    vkBeginCommandBuffer(mCmdBuf, &beginInfo);
    vkCmdBindPipeline(mCmdBuf, VK_PIPELINE_BIND_POINT_COMPUTE, mPipeline);
    vkCmdBindDescriptorSets(mCmdBuf, VK_PIPELINE_BIND_POINT_COMPUTE,
                            mPipeLayout, 0, 1, &mDescSet, 0, NULL);
    vkCmdDispatch(mCmdBuf, (width + 15) / 16, (height + 15) / 16, 1);
    vkEndCommandBuffer(mCmdBuf);

    VkSubmitInfo si = {};
    si.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    si.commandBufferCount = 1;
    si.pCommandBuffers = &mCmdBuf;
    vkQueueSubmit(mQueue, 1, &si, VK_NULL_HANDLE);
    vkQueueWaitIdle(mQueue);
    int64_t t2 = nowMs();

    /* Invalidate + readback via persistent map */
    VkMappedMemoryRange outRange = {};
    outRange.sType = VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE;
    outRange.memory = mOutMem;
    outRange.size = VK_WHOLE_SIZE;
    vkInvalidateMappedMemoryRanges(mDevice, 1, &outRange);

    memcpy(dst, mOutMap, mOutSize);
    int64_t t3 = nowMs();

    ALOGD("VK DMA: params=%lld gpu=%lld readback=%lldms (upload=0)", t1 - t0, t2 - t1, t3 - t2);

    updateAwb(cpuFallback, width, height, is16, pixFmt);
    return true;
}

/* --- cleanup --- */

void VulkanIspPipeline::destroy() {
    if (!mReady && mDevice == VK_NULL_HANDLE)
        return;

    if (mDevice != VK_NULL_HANDLE) {
        vkDeviceWaitIdle(mDevice);
        /* Unmap persistent maps before destroying memory */
        if (mInMap) { vkUnmapMemory(mDevice, mInMem); mInMap = NULL; }
        if (mOutMap) { vkUnmapMemory(mDevice, mOutMem); mOutMap = NULL; }
        if (mParamMap) { vkUnmapMemory(mDevice, mParamMem); mParamMap = NULL; }
        /* Destroy buffers */
        destroyBuffer(mInBuf, mInMem);
        destroyBuffer(mOutBuf, mOutMem);
        destroyBuffer(mParamBuf, mParamMem);
        if (mDmaBuf != VK_NULL_HANDLE) destroyBuffer(mDmaBuf, mDmaMem);
        mDmaBuf = VK_NULL_HANDLE; mDmaMem = VK_NULL_HANDLE; mDmaFd = -1;
        mInBuf = VK_NULL_HANDLE; mInMem = VK_NULL_HANDLE;
        mOutBuf = VK_NULL_HANDLE; mOutMem = VK_NULL_HANDLE;
        mParamBuf = VK_NULL_HANDLE; mParamMem = VK_NULL_HANDLE;
        if (mCmdPool) { vkDestroyCommandPool(mDevice, mCmdPool, NULL); mCmdPool = VK_NULL_HANDLE; }
        if (mDescPool) { vkDestroyDescriptorPool(mDevice, mDescPool, NULL); mDescPool = VK_NULL_HANDLE; }
        if (mPipeline) { vkDestroyPipeline(mDevice, mPipeline, NULL); mPipeline = VK_NULL_HANDLE; }
        if (mPipeLayout) { vkDestroyPipelineLayout(mDevice, mPipeLayout, NULL); mPipeLayout = VK_NULL_HANDLE; }
        if (mDescLayout) { vkDestroyDescriptorSetLayout(mDevice, mDescLayout, NULL); mDescLayout = VK_NULL_HANDLE; }
        if (mShader) { vkDestroyShaderModule(mDevice, mShader, NULL); mShader = VK_NULL_HANDLE; }
        vkDestroyDevice(mDevice, NULL); mDevice = VK_NULL_HANDLE;
    }
    if (mInstance) { vkDestroyInstance(mInstance, NULL); mInstance = VK_NULL_HANDLE; }
    mReady = false;
    mInSize = 0; mOutSize = 0;
    mBufWidth = 0; mBufHeight = 0;
}

}; /* namespace android */
