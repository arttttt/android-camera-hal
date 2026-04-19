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

#include <system/window.h>
#include "VulkanIspPipeline.h"
#include "VulkanLoader.h"
#include "VulkanPfn.h"

/* VK_ANDROID_native_buffer types — absent from android-24 NDK vulkan.h */
#ifndef VK_ANDROID_NATIVE_BUFFER_NUMBER
#define VK_ANDROID_NATIVE_BUFFER_NUMBER 11
#define VK_STRUCTURE_TYPE_NATIVE_BUFFER_ANDROID ((VkStructureType)1000010000)
typedef struct VkNativeBufferANDROID {
    VkStructureType sType;
    const void *pNext;
    const struct native_handle *handle;
    int stride;
    int format;
    int usage;
} VkNativeBufferANDROID;
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
    : mLoader(NULL), mPfn(NULL)
    , mReady(false), mBufWidth(0), mBufHeight(0)
    , mInstance(VK_NULL_HANDLE), mPhysDev(VK_NULL_HANDLE)
    , mDevice(VK_NULL_HANDLE), mQueue(VK_NULL_HANDLE), mQueueFamily(0)
    , mShader(VK_NULL_HANDLE), mVertShader(VK_NULL_HANDLE), mFragShader(VK_NULL_HANDLE)
    , mDescLayout(VK_NULL_HANDLE)
    , mPipeLayout(VK_NULL_HANDLE), mPipeline(VK_NULL_HANDLE)
    , mBlitPipeline(VK_NULL_HANDLE), mRenderPass(VK_NULL_HANDLE)
    , mDescPool(VK_NULL_HANDLE), mDescSet(VK_NULL_HANDLE)
    , mCmdPool(VK_NULL_HANDLE), mCmdBuf(VK_NULL_HANDLE)
    , mInBuf(VK_NULL_HANDLE), mOutBuf(VK_NULL_HANDLE), mParamBuf(VK_NULL_HANDLE)
    , mInMem(VK_NULL_HANDLE), mOutMem(VK_NULL_HANDLE), mParamMem(VK_NULL_HANDLE)
    , mInSize(0), mOutSize(0)
    , mInMap(NULL), mOutMap(NULL), mParamMap(NULL)
    , mScratchImg(VK_NULL_HANDLE), mScratchMem(VK_NULL_HANDLE), mScratchView(VK_NULL_HANDLE)
    , mFence(VK_NULL_HANDLE), mPrevPending(false)
    , mParamsTemplateReady(false)
    , mNativeBufferAvail(false) {}

VulkanIspPipeline::~VulkanIspPipeline() { destroy(); }

/* --- helpers (no VK calls) --- */

void VulkanIspPipeline::fillParams(IspParams *p, unsigned w, unsigned h,
                                    bool is16, uint32_t pixFmt) {
    if (!mParamsTemplateReady) {
        memset(&mParamsTemplate, 0, sizeof(IspParams));
        if (mCcm) {
            for (int i = 0; i < 9; i++) mParamsTemplate.ccm[i] = mCcm[i];
        } else {
            mParamsTemplate.ccm[0] = 1024; mParamsTemplate.ccm[4] = 1024; mParamsTemplate.ccm[8] = 1024;
        }
        mParamsTemplateReady = true;
    }

    *p = mParamsTemplate;
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
}

void VulkanIspPipeline::updateAwb(const uint8_t *raw, unsigned w, unsigned h,
                                    bool is16, uint32_t pixFmt) {
    if (!mEnabled || !raw || mAwbLocked) return;

    uint64_t sR = 0, sG = 0, sB = 0, nR = 0, nG = 0, nB = 0;
    unsigned rX = (pixFmt == V4L2_PIX_FMT_SGRBG10 || pixFmt == V4L2_PIX_FMT_SGRBG8 ||
                   pixFmt == V4L2_PIX_FMT_SBGGR10 || pixFmt == V4L2_PIX_FMT_SBGGR8) ? 1 : 0;
    unsigned rY = (pixFmt == V4L2_PIX_FMT_SGBRG10 || pixFmt == V4L2_PIX_FMT_SGBRG8 ||
                   pixFmt == V4L2_PIX_FMT_SBGGR10 || pixFmt == V4L2_PIX_FMT_SBGGR8) ? 1 : 0;

    for (unsigned y = 0; y < h; y += 8) {
        for (unsigned x = 0; x < w; x += 8) {
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
        const float alpha = 0.15f;
        mWbR = (unsigned)(alpha * r + (1.0f - alpha) * mWbR);
        mWbG = (unsigned)(alpha * g + (1.0f - alpha) * mWbG);
        mWbB = (unsigned)(alpha * b + (1.0f - alpha) * mWbB);
    }
}

/* --- Vulkan buffer management --- */

uint32_t VulkanIspPipeline::findMemoryType(uint32_t filter, VkMemoryPropertyFlags props) {
    VkPhysicalDeviceMemoryProperties memProps;
    mPfn->GetPhysicalDeviceMemoryProperties(mPhysDev, &memProps);
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

    if (mPfn->CreateBuffer(mDevice, &ci, NULL, buf) != VK_SUCCESS)
        return false;

    VkMemoryRequirements req;
    mPfn->GetBufferMemoryRequirements(mDevice, *buf, &req);

    VkMemoryAllocateInfo ai = {};
    ai.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    ai.allocationSize = req.size;
    ai.memoryTypeIndex = findMemoryType(req.memoryTypeBits,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_CACHED_BIT);
    if (ai.memoryTypeIndex == UINT32_MAX)
        ai.memoryTypeIndex = findMemoryType(req.memoryTypeBits,
            VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);

    if (ai.memoryTypeIndex == UINT32_MAX ||
        mPfn->AllocateMemory(mDevice, &ai, NULL, mem) != VK_SUCCESS) {
        mPfn->DestroyBuffer(mDevice, *buf, NULL);
        *buf = VK_NULL_HANDLE;
        return false;
    }

    mPfn->BindBufferMemory(mDevice, *buf, *mem, 0);
    return true;
}

void VulkanIspPipeline::destroyBuffer(VkBuffer buf, VkDeviceMemory mem) {
    if (buf != VK_NULL_HANDLE) mPfn->DestroyBuffer(mDevice, buf, NULL);
    if (mem != VK_NULL_HANDLE) mPfn->FreeMemory(mDevice, mem, NULL);
}

void VulkanIspPipeline::recordAndSubmit(unsigned width, unsigned height, VkFence fence,
                                         bool copyToOutBuf) {
    VkCommandBufferBeginInfo beginInfo = {};
    beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

    mPfn->ResetCommandBuffer(mCmdBuf, 0);
    mPfn->BeginCommandBuffer(mCmdBuf, &beginInfo);
    mPfn->CmdBindPipeline(mCmdBuf, VK_PIPELINE_BIND_POINT_COMPUTE, mPipeline);
    mPfn->CmdBindDescriptorSets(mCmdBuf, VK_PIPELINE_BIND_POINT_COMPUTE,
                                mPipeLayout, 0, 1, &mDescSet, 0, NULL);
    mPfn->CmdDispatch(mCmdBuf, (width + 7) / 8, (height + 7) / 8, 1);

    if (copyToOutBuf) {
        VkImageMemoryBarrier imb = {};
        imb.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
        imb.oldLayout = VK_IMAGE_LAYOUT_GENERAL;
        imb.newLayout = VK_IMAGE_LAYOUT_GENERAL;
        imb.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        imb.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
        imb.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        imb.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        imb.image = mScratchImg;
        imb.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        imb.subresourceRange.levelCount = 1;
        imb.subresourceRange.layerCount = 1;

        mPfn->CmdPipelineBarrier(mCmdBuf,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT,
            0, 0, NULL, 0, NULL, 1, &imb);

        VkBufferImageCopy region = {};
        region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        region.imageSubresource.layerCount = 1;
        region.imageExtent.width = width;
        region.imageExtent.height = height;
        region.imageExtent.depth = 1;
        mPfn->CmdCopyImageToBuffer(mCmdBuf, mScratchImg, VK_IMAGE_LAYOUT_GENERAL,
                                    mOutBuf, 1, &region);
    }

    mPfn->EndCommandBuffer(mCmdBuf);

    VkSubmitInfo si = {};
    si.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    si.commandBufferCount = 1;
    si.pCommandBuffers = &mCmdBuf;
    mPfn->QueueSubmit(mQueue, 1, &si, fence);
}

/* --- init / destroy --- */

bool VulkanIspPipeline::init() {
    if (mReady) return true;

    mLoader = createVulkanLoader();
    if (!mLoader->load()) {
        ALOGE("VulkanLoader '%s' load() failed", mLoader->name());
        destroy();
        return false;
    }
    ALOGD("VulkanLoader: %s", mLoader->name());

    mPfn = new VulkanPfn();
    memset(mPfn, 0, sizeof(*mPfn));

    VkApplicationInfo appInfo = {};
    appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
    appInfo.pApplicationName = "CameraISP";
    appInfo.apiVersion = VK_API_VERSION;

    /* Enumerate + enable instance extensions relevant to external memory. */
    bool hasGpdp2 = false, hasExtMemCaps = false, hasNvExtMemCaps = false;
    {
        PFN_vkEnumerateInstanceExtensionProperties gieep =
            mLoader->getEnumerateInstanceExtensionProperties();
        uint32_t n = 0;
        gieep(NULL, &n, NULL);
        VkExtensionProperties *ex = new VkExtensionProperties[n];
        gieep(NULL, &n, ex);
        for (uint32_t i = 0; i < n; i++) {
            if (strstr(ex[i].extensionName, "external") ||
                strstr(ex[i].extensionName, "get_physical_device_properties2") ||
                strstr(ex[i].extensionName, "VK_NV_"))
                ALOGD("Instance ext: %s", ex[i].extensionName);
            if (!strcmp(ex[i].extensionName, "VK_KHR_get_physical_device_properties2"))
                hasGpdp2 = true;
            if (!strcmp(ex[i].extensionName, "VK_KHR_external_memory_capabilities"))
                hasExtMemCaps = true;
            if (!strcmp(ex[i].extensionName, "VK_NV_external_memory_capabilities"))
                hasNvExtMemCaps = true;
        }
        delete[] ex;
    }

    const char *iexts[8];
    uint32_t iec = 0;
    if (hasGpdp2)       iexts[iec++] = "VK_KHR_get_physical_device_properties2";
    if (hasExtMemCaps)  iexts[iec++] = "VK_KHR_external_memory_capabilities";
    if (hasNvExtMemCaps)iexts[iec++] = "VK_NV_external_memory_capabilities";

    VkInstanceCreateInfo ici = {};
    ici.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
    ici.pApplicationInfo = &appInfo;
    ici.enabledExtensionCount = iec;
    ici.ppEnabledExtensionNames = iexts;

    if (mLoader->getCreateInstance()(&ici, NULL, &mInstance) != VK_SUCCESS) {
        ALOGE("vkCreateInstance failed");
        destroy();
        return false;
    }

    mLoader->loadInstancePfns(mInstance, mPfn);
    if (!mPfn->DestroyInstance || !mPfn->EnumeratePhysicalDevices ||
        !mPfn->CreateDevice    || !mPfn->GetDeviceProcAddr) {
        ALOGE("Instance-level PFN resolution failed");
        destroy();
        return false;
    }

    /* Physical device */
    uint32_t devCount = 1;
    if (mPfn->EnumeratePhysicalDevices(mInstance, &devCount, &mPhysDev) != VK_SUCCESS || devCount == 0) {
        ALOGE("No Vulkan physical device");
        destroy();
        return false;
    }

    /* Enumerate device extensions. */
    bool hasNvGlsl = false;
    bool hasKhrExtMem = false, hasKhrExtMemFd = false;
    bool hasExtExtMemDmaBuf = false, hasNvExtMem = false;
    mNativeBufferAvail = false;
    {
        uint32_t extCount = 0;
        mPfn->EnumerateDeviceExtensionProperties(mPhysDev, NULL, &extCount, NULL);
        VkExtensionProperties *exts = new VkExtensionProperties[extCount];
        mPfn->EnumerateDeviceExtensionProperties(mPhysDev, NULL, &extCount, exts);
        for (uint32_t i = 0; i < extCount; i++) {
            ALOGD("VK ext: %s", exts[i].extensionName);
            if (!strcmp(exts[i].extensionName, "VK_NV_glsl_shader"))
                hasNvGlsl = true;
            if (!strcmp(exts[i].extensionName, "VK_ANDROID_native_buffer"))
                mNativeBufferAvail = true;
            if (!strcmp(exts[i].extensionName, "VK_KHR_external_memory"))
                hasKhrExtMem = true;
            if (!strcmp(exts[i].extensionName, "VK_KHR_external_memory_fd"))
                hasKhrExtMemFd = true;
            if (!strcmp(exts[i].extensionName, "VK_EXT_external_memory_dma_buf"))
                hasExtExtMemDmaBuf = true;
            if (!strcmp(exts[i].extensionName, "VK_NV_external_memory"))
                hasNvExtMem = true;
        }
        delete[] exts;
    }
    ALOGD("External memory device ext: KHR_external_memory=%d fd=%d dma_buf=%d NV=%d",
          hasKhrExtMem, hasKhrExtMemFd, hasExtExtMemDmaBuf, hasNvExtMem);
    if (!hasNvGlsl) {
        ALOGE("VK_NV_glsl_shader not supported — Vulkan ISP unavailable");
        destroy();
        return false;
    }
    ALOGD("VK_ANDROID_native_buffer: %s", mNativeBufferAvail ? "AVAILABLE" : "absent");

    /* Compute queue */
    uint32_t qfCount = 0;
    mPfn->GetPhysicalDeviceQueueFamilyProperties(mPhysDev, &qfCount, NULL);
    VkQueueFamilyProperties qfProps[8];
    if (qfCount > 8) qfCount = 8;
    mPfn->GetPhysicalDeviceQueueFamilyProperties(mPhysDev, &qfCount, qfProps);

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

    /* Logical device */
    float qPriority = 1.0f;
    VkDeviceQueueCreateInfo qci = {};
    qci.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
    qci.queueFamilyIndex = mQueueFamily;
    qci.queueCount = 1;
    qci.pQueuePriorities = &qPriority;

    const char *enabledExts[8];
    uint32_t enabledExtCount = 0;
    enabledExts[enabledExtCount++] = "VK_NV_glsl_shader";
    if (mNativeBufferAvail)   enabledExts[enabledExtCount++] = "VK_ANDROID_native_buffer";
    if (hasKhrExtMem)         enabledExts[enabledExtCount++] = "VK_KHR_external_memory";
    if (hasKhrExtMemFd)       enabledExts[enabledExtCount++] = "VK_KHR_external_memory_fd";
    if (hasExtExtMemDmaBuf)   enabledExts[enabledExtCount++] = "VK_EXT_external_memory_dma_buf";
    if (hasNvExtMem)          enabledExts[enabledExtCount++] = "VK_NV_external_memory";

    VkDeviceCreateInfo dci = {};
    dci.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
    dci.queueCreateInfoCount = 1;
    dci.pQueueCreateInfos = &qci;
    dci.enabledExtensionCount = enabledExtCount;
    dci.ppEnabledExtensionNames = enabledExts;

    if (mPfn->CreateDevice(mPhysDev, &dci, NULL, &mDevice) != VK_SUCCESS) {
        ALOGE("vkCreateDevice failed");
        destroy();
        return false;
    }

    mLoader->loadDevicePfns(mDevice, mPfn);
    if (!mPfn->QueueSubmit || !mPfn->CreateBuffer || !mPfn->CreateShaderModule) {
        ALOGE("Device-level PFN resolution failed (critical fn missing)");
        destroy();
        return false;
    }

    mPfn->GetDeviceQueue(mDevice, mQueueFamily, 0, &mQueue);

    /* Shader module — GLSL source via VK_NV_glsl_shader */
    static const char *kGlslShaderSrc =
        "#version 450\n"
        "layout(local_size_x = 8, local_size_y = 8) in;\n"
        "layout(std430, binding = 0) buffer InputBuf  { uint data[]; } inBuf;\n"
        "layout(rgba8, binding = 1) writeonly uniform image2D outImg;\n"
        "layout(std430, binding = 2) buffer Params {\n"
        "    uint width; uint height; uint bayerPhase; uint is16bit;\n"
        "    uint wbR; uint wbG; uint wbB; uint doIsp;\n"
        "    int ccm[9]; uint gammaLut[64];\n"
        "} params;\n"
        "uint readPixel(uint x, uint y) {\n"
        "    if (params.is16bit != 0u) {\n"
        "        uint idx = y * params.width + x;\n"
        "        uint word = inBuf.data[idx >> 1u];\n"
        "        uint val = ((idx & 1u) == 0u) ? (word & 0xFFFFu) : (word >> 16);\n"
        "        return val >> 2;\n"
        "    } else {\n"
        "        uint idx = y * params.width + x;\n"
        "        uint word = inBuf.data[idx >> 2u];\n"
        "        return (word >> ((idx & 3u) * 8u)) & 0xFFu;\n"
        "    }\n"
        "}\n"
        "float srgbGamma(float lin) {\n"
        "    return (lin <= 0.0031308) ? lin * 12.92 : 1.055 * pow(lin, 1.0/2.4) - 0.055;\n"
        "}\n"
        "void main() {\n"
        "    uint x = gl_GlobalInvocationID.x, y = gl_GlobalInvocationID.y;\n"
        "    if (x >= params.width || y >= params.height) return;\n"
        "\n"
        "    /* McGuire/Malvar-He-Cutler 5x5 demosaic — 13 samples, branch-free */\n"
        "    int ix = int(x), iy = int(y);\n"
        "    int iw = int(params.width) - 1, ih = int(params.height) - 1;\n"
        "#define PX(dx, dy) float(readPixel(uint(clamp(ix+(dx), 0, iw)), uint(clamp(iy+(dy), 0, ih))))\n"
        "    float pC  = PX(0, 0);\n"
        "    float pN  = PX(0,-1);  float pS  = PX(0, 1);\n"
        "    float pW  = PX(-1, 0); float pE  = PX(1, 0);\n"
        "    float pN2 = PX(0,-2);  float pS2 = PX(0, 2);\n"
        "    float pW2 = PX(-2, 0); float pE2 = PX(2, 0);\n"
        "    float pNW = PX(-1,-1); float pNE = PX(1,-1);\n"
        "    float pSW = PX(-1, 1); float pSE = PX(1, 1);\n"
        "#undef PX\n"
        "    float vFar  = pN2 + pS2;\n"
        "    float vNear = pN + pS;\n"
        "    float diag  = pNW + pNE + pSW + pSE;\n"
        "    float hFar  = pW2 + pE2;\n"
        "    float hNear = pW + pE;\n"
        "\n"
        "    float Pcross = (4.0*pC - vFar + 2.0*vNear - hFar + 2.0*hNear) * 0.125;\n"
        "    float Pcheck = (6.0*pC - 1.5*vFar + 2.0*diag - 1.5*hFar) * 0.125;\n"
        "    float Ptheta = (5.0*pC + 0.5*vFar - diag - hFar + 4.0*hNear) * 0.125;\n"
        "    float Pphi   = (5.0*pC - vFar - diag + 0.5*hFar + 4.0*vNear) * 0.125;\n"
        "\n"
        "    /* Branch-free Bayer position selection */\n"
        "    uint rX = params.bayerPhase & 1u;\n"
        "    uint rY = (params.bayerPhase >> 1) & 1u;\n"
        "    bool isRedRow = ((y + rY) & 1u) == 0u;\n"
        "    bool isRedCol = ((x + rX) & 1u) == 0u;\n"
        "    float fR = isRedRow ? (isRedCol ? pC : Ptheta) : (isRedCol ? Pphi : Pcheck);\n"
        "    float fG = (isRedRow == isRedCol) ? Pcross : pC;\n"
        "    float fB = isRedRow ? (isRedCol ? Pcheck : Pphi) : (isRedCol ? Ptheta : pC);\n"
        "    int R = clamp(int(fR + 0.5), 0, 255);\n"
        "    int G = clamp(int(fG + 0.5), 0, 255);\n"
        "    int B = clamp(int(fB + 0.5), 0, 255);\n"
        "\n"
        "    if (params.doIsp != 0u) {\n"
        "        R = clamp((R * int(params.wbR)) >> 8, 0, 255);\n"
        "        G = clamp((G * int(params.wbG)) >> 8, 0, 255);\n"
        "        B = clamp((B * int(params.wbB)) >> 8, 0, 255);\n"
        "        int rr = clamp((params.ccm[0]*R + params.ccm[1]*G + params.ccm[2]*B) >> 10, 0, 255);\n"
        "        int gg = clamp((params.ccm[3]*R + params.ccm[4]*G + params.ccm[5]*B) >> 10, 0, 255);\n"
        "        int bb = clamp((params.ccm[6]*R + params.ccm[7]*G + params.ccm[8]*B) >> 10, 0, 255);\n"
        "        R = clamp(int(srgbGamma(float(rr)/255.0) * 255.0 + 0.5), 0, 255);\n"
        "        G = clamp(int(srgbGamma(float(gg)/255.0) * 255.0 + 0.5), 0, 255);\n"
        "        B = clamp(int(srgbGamma(float(bb)/255.0) * 255.0 + 0.5), 0, 255);\n"
        "    }\n"
        "    imageStore(outImg, ivec2(int(x), int(y)),\n"
        "               vec4(float(R), float(G), float(B), 255.0) / 255.0);\n"
        "}\n";

    VkShaderModuleCreateInfo smi = {};
    smi.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    smi.codeSize = strlen(kGlslShaderSrc);
    smi.pCode = (const uint32_t *)kGlslShaderSrc;

    if (mPfn->CreateShaderModule(mDevice, &smi, NULL, &mShader) != VK_SUCCESS) {
        ALOGE("vkCreateShaderModule failed");
        destroy();
        return false;
    }

    /* Descriptor set layout — binding 0/2 = storage buffer (compute reads Bayer
     * + params), binding 1 = storage image (compute writes, fragment reads). */
    VkDescriptorSetLayoutBinding bindings[3] = {};
    bindings[0].binding = 0;
    bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    bindings[0].descriptorCount = 1;
    bindings[0].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    bindings[1] = bindings[0]; bindings[1].binding = 1;
    bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
    bindings[1].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    bindings[2] = bindings[0]; bindings[2].binding = 2;

    VkDescriptorSetLayoutCreateInfo dslci = {};
    dslci.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    dslci.bindingCount = 3;
    dslci.pBindings = bindings;
    if (mPfn->CreateDescriptorSetLayout(mDevice, &dslci, NULL, &mDescLayout) != VK_SUCCESS) {
        ALOGE("vkCreateDescriptorSetLayout failed");
        destroy(); return false;
    }

    /* Pipeline layout */
    VkPipelineLayoutCreateInfo plci = {};
    plci.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    plci.setLayoutCount = 1;
    plci.pSetLayouts = &mDescLayout;
    if (mPfn->CreatePipelineLayout(mDevice, &plci, NULL, &mPipeLayout) != VK_SUCCESS) {
        ALOGE("vkCreatePipelineLayout failed");
        destroy(); return false;
    }

    /* Compute pipeline */
    VkComputePipelineCreateInfo cpci = {};
    cpci.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
    cpci.stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    cpci.stage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
    cpci.stage.module = mShader;
    cpci.stage.pName = "main";
    cpci.layout = mPipeLayout;

    if (mPfn->CreateComputePipelines(mDevice, VK_NULL_HANDLE, 1, &cpci, NULL, &mPipeline) != VK_SUCCESS) {
        ALOGE("vkCreateComputePipelines failed");
        destroy();
        return false;
    }

    /* Graphics pipeline — blits scratch image → gralloc colour attachment
     * through the driver's ROP path, which is the only write path that
     * knows about nvgralloc's blocklinear layout on Tegra. */
    static const char *kVertexShaderSrc =
        "#version 450\n"
        "void main() {\n"
        "    float x = float((gl_VertexIndex << 1) & 2) * 2.0 - 1.0;\n"
        "    float y = float(gl_VertexIndex & 2) * 2.0 - 1.0;\n"
        "    gl_Position = vec4(x, y, 0.0, 1.0);\n"
        "}\n";
    static const char *kFragmentShaderSrc =
        "#version 450\n"
        "layout(rgba8, binding = 1) uniform readonly image2D scratchImg;\n"
        "layout(location = 0) out vec4 outColor;\n"
        "void main() {\n"
        "    outColor = imageLoad(scratchImg, ivec2(gl_FragCoord.xy));\n"
        "}\n";

    VkShaderModuleCreateInfo vsmi = {};
    vsmi.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    vsmi.codeSize = strlen(kVertexShaderSrc);
    vsmi.pCode = (const uint32_t *)kVertexShaderSrc;
    if (mPfn->CreateShaderModule(mDevice, &vsmi, NULL, &mVertShader) != VK_SUCCESS) {
        ALOGE("vertex vkCreateShaderModule failed");
        destroy(); return false;
    }
    vsmi.codeSize = strlen(kFragmentShaderSrc);
    vsmi.pCode = (const uint32_t *)kFragmentShaderSrc;
    if (mPfn->CreateShaderModule(mDevice, &vsmi, NULL, &mFragShader) != VK_SUCCESS) {
        ALOGE("fragment vkCreateShaderModule failed");
        destroy(); return false;
    }

    /* Render pass — single RGBA8 colour attachment, DONT_CARE→STORE. */
    VkAttachmentDescription att = {};
    att.format         = VK_FORMAT_R8G8B8A8_UNORM;
    att.samples        = VK_SAMPLE_COUNT_1_BIT;
    att.loadOp         = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    att.storeOp        = VK_ATTACHMENT_STORE_OP_STORE;
    att.stencilLoadOp  = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    att.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    att.initialLayout  = VK_IMAGE_LAYOUT_UNDEFINED;
    att.finalLayout    = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    VkAttachmentReference attRef = {};
    attRef.attachment = 0;
    attRef.layout     = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    VkSubpassDescription subpass = {};
    subpass.pipelineBindPoint    = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass.colorAttachmentCount = 1;
    subpass.pColorAttachments    = &attRef;

    VkRenderPassCreateInfo rpci = {};
    rpci.sType           = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
    rpci.attachmentCount = 1;
    rpci.pAttachments    = &att;
    rpci.subpassCount    = 1;
    rpci.pSubpasses      = &subpass;
    if (mPfn->CreateRenderPass(mDevice, &rpci, NULL, &mRenderPass) != VK_SUCCESS) {
        ALOGE("vkCreateRenderPass failed");
        destroy(); return false;
    }

    /* Graphics pipeline. */
    VkPipelineShaderStageCreateInfo stages[2] = {};
    stages[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
    stages[0].module = mVertShader;
    stages[0].pName = "main";
    stages[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    stages[1].module = mFragShader;
    stages[1].pName = "main";

    VkPipelineVertexInputStateCreateInfo vis = {};
    vis.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;

    VkPipelineInputAssemblyStateCreateInfo ias = {};
    ias.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
    ias.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

    VkPipelineViewportStateCreateInfo vps = {};
    vps.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
    vps.viewportCount = 1;
    vps.scissorCount  = 1;

    VkPipelineRasterizationStateCreateInfo rs = {};
    rs.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
    rs.polygonMode = VK_POLYGON_MODE_FILL;
    rs.cullMode    = VK_CULL_MODE_NONE;
    rs.frontFace   = VK_FRONT_FACE_COUNTER_CLOCKWISE;
    rs.lineWidth   = 1.0f;

    VkPipelineMultisampleStateCreateInfo ms = {};
    ms.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
    ms.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

    VkPipelineColorBlendAttachmentState cba = {};
    cba.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
                         VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;

    VkPipelineColorBlendStateCreateInfo cbs = {};
    cbs.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
    cbs.attachmentCount = 1;
    cbs.pAttachments    = &cba;

    VkDynamicState dyns[2] = { VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR };
    VkPipelineDynamicStateCreateInfo ds = {};
    ds.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
    ds.dynamicStateCount = 2;
    ds.pDynamicStates    = dyns;

    VkGraphicsPipelineCreateInfo gpci = {};
    gpci.sType               = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
    gpci.stageCount          = 2;
    gpci.pStages             = stages;
    gpci.pVertexInputState   = &vis;
    gpci.pInputAssemblyState = &ias;
    gpci.pViewportState      = &vps;
    gpci.pRasterizationState = &rs;
    gpci.pMultisampleState   = &ms;
    gpci.pColorBlendState    = &cbs;
    gpci.pDynamicState       = &ds;
    gpci.layout              = mPipeLayout;
    gpci.renderPass          = mRenderPass;
    gpci.subpass             = 0;

    if (mPfn->CreateGraphicsPipelines(mDevice, VK_NULL_HANDLE, 1, &gpci, NULL,
                                        &mBlitPipeline) != VK_SUCCESS) {
        ALOGE("vkCreateGraphicsPipelines failed");
        destroy();
        return false;
    }

    /* Descriptor pool — 2 storage buffers (input, params) + 1 storage image (output) */
    VkDescriptorPoolSize poolSizes[2] = {};
    poolSizes[0].type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    poolSizes[0].descriptorCount = 2;
    poolSizes[1].type = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
    poolSizes[1].descriptorCount = 1;

    VkDescriptorPoolCreateInfo dpci = {};
    dpci.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    dpci.maxSets = 1;
    dpci.poolSizeCount = 2;
    dpci.pPoolSizes = poolSizes;
    if (mPfn->CreateDescriptorPool(mDevice, &dpci, NULL, &mDescPool) != VK_SUCCESS) {
        ALOGE("vkCreateDescriptorPool failed");
        destroy(); return false;
    }

    VkDescriptorSetAllocateInfo dsai = {};
    dsai.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    dsai.descriptorPool = mDescPool;
    dsai.descriptorSetCount = 1;
    dsai.pSetLayouts = &mDescLayout;
    if (mPfn->AllocateDescriptorSets(mDevice, &dsai, &mDescSet) != VK_SUCCESS) {
        ALOGE("vkAllocateDescriptorSets failed");
        destroy(); return false;
    }

    /* Command pool + buffer */
    VkCommandPoolCreateInfo cpi = {};
    cpi.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    cpi.queueFamilyIndex = mQueueFamily;
    cpi.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    mPfn->CreateCommandPool(mDevice, &cpi, NULL, &mCmdPool);

    VkCommandBufferAllocateInfo cbai = {};
    cbai.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    cbai.commandPool = mCmdPool;
    cbai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    cbai.commandBufferCount = 1;
    mPfn->AllocateCommandBuffers(mDevice, &cbai, &mCmdBuf);

    /* Params buffer — persistent map */
    if (!createBuffer(&mParamBuf, &mParamMem, sizeof(IspParams),
                      VK_BUFFER_USAGE_STORAGE_BUFFER_BIT)) {
        ALOGE("Failed to create params buffer");
        destroy();
        return false;
    }
    mPfn->MapMemory(mDevice, mParamMem, 0, sizeof(IspParams), 0, &mParamMap);

    VkFenceCreateInfo fci = {};
    fci.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    mPfn->CreateFence(mDevice, &fci, NULL, &mFence);

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

    if (!recreate) return true;

    /* Any cached gralloc images were sized for old resolution — drop them. */
    clearGrallocImages();

    if (mInMap)  { mPfn->UnmapMemory(mDevice, mInMem);  mInMap = NULL; }
    if (mOutMap) { mPfn->UnmapMemory(mDevice, mOutMem); mOutMap = NULL; }

    destroyBuffer(mInBuf, mInMem);   mInBuf = VK_NULL_HANDLE;  mInMem = VK_NULL_HANDLE;
    destroyBuffer(mOutBuf, mOutMem); mOutBuf = VK_NULL_HANDLE; mOutMem = VK_NULL_HANDLE;

    if (mScratchView) { mPfn->DestroyImageView(mDevice, mScratchView, NULL); mScratchView = VK_NULL_HANDLE; }
    if (mScratchImg)  { mPfn->DestroyImage(mDevice, mScratchImg, NULL);      mScratchImg = VK_NULL_HANDLE; }
    if (mScratchMem)  { mPfn->FreeMemory(mDevice, mScratchMem, NULL);        mScratchMem = VK_NULL_HANDLE; }

    clearGrallocImages();

    if (!createBuffer(&mInBuf,  &mInMem,  inSize,  VK_BUFFER_USAGE_STORAGE_BUFFER_BIT) ||
        !createBuffer(&mOutBuf, &mOutMem, outSize,
                      VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                      VK_BUFFER_USAGE_TRANSFER_SRC_BIT)) {
        ALOGE("Failed to allocate Vulkan buffers %ux%u", width, height);
        return false;
    }

    mPfn->MapMemory(mDevice, mInMem,  0, inSize,  0, &mInMap);
    mPfn->MapMemory(mDevice, mOutMem, 0, outSize, 0, &mOutMap);

    mInSize = inSize; mOutSize = outSize;
    mBufWidth = width; mBufHeight = height;

    VkImageCreateInfo ici = {};
    ici.sType         = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    ici.imageType     = VK_IMAGE_TYPE_2D;
    ici.format        = VK_FORMAT_R8G8B8A8_UNORM;
    ici.extent.width  = width;
    ici.extent.height = height;
    ici.extent.depth  = 1;
    ici.mipLevels     = 1;
    ici.arrayLayers   = 1;
    ici.samples       = VK_SAMPLE_COUNT_1_BIT;
    ici.tiling        = VK_IMAGE_TILING_OPTIMAL;
    ici.usage         = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
    ici.sharingMode   = VK_SHARING_MODE_EXCLUSIVE;
    ici.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

    if (mPfn->CreateImage(mDevice, &ici, NULL, &mScratchImg) != VK_SUCCESS) {
        ALOGE("Scratch vkCreateImage failed %ux%u", width, height);
        return false;
    }

    VkMemoryRequirements req;
    mPfn->GetImageMemoryRequirements(mDevice, mScratchImg, &req);

    VkMemoryAllocateInfo ai = {};
    ai.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    ai.allocationSize = req.size;
    ai.memoryTypeIndex = findMemoryType(req.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    if (ai.memoryTypeIndex == UINT32_MAX)
        ai.memoryTypeIndex = findMemoryType(req.memoryTypeBits, 0);

    if (ai.memoryTypeIndex == UINT32_MAX ||
        mPfn->AllocateMemory(mDevice, &ai, NULL, &mScratchMem) != VK_SUCCESS) {
        ALOGE("Scratch vkAllocateMemory failed");
        return false;
    }
    mPfn->BindImageMemory(mDevice, mScratchImg, mScratchMem, 0);

    VkImageViewCreateInfo vci = {};
    vci.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    vci.image = mScratchImg;
    vci.viewType = VK_IMAGE_VIEW_TYPE_2D;
    vci.format = VK_FORMAT_R8G8B8A8_UNORM;
    vci.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    vci.subresourceRange.levelCount = 1;
    vci.subresourceRange.layerCount = 1;
    if (mPfn->CreateImageView(mDevice, &vci, NULL, &mScratchView) != VK_SUCCESS) {
        ALOGE("Scratch vkCreateImageView failed");
        return false;
    }

    /* One-time UNDEFINED → GENERAL transition. */
    VkCommandBufferBeginInfo bi = {};
    bi.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    mPfn->ResetCommandBuffer(mCmdBuf, 0);
    mPfn->BeginCommandBuffer(mCmdBuf, &bi);

    VkImageMemoryBarrier imb = {};
    imb.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    imb.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    imb.newLayout = VK_IMAGE_LAYOUT_GENERAL;
    imb.srcAccessMask = 0;
    imb.dstAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
    imb.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    imb.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    imb.image = mScratchImg;
    imb.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    imb.subresourceRange.levelCount = 1;
    imb.subresourceRange.layerCount = 1;

    mPfn->CmdPipelineBarrier(mCmdBuf,
        VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
        0, 0, NULL, 0, NULL, 1, &imb);
    mPfn->EndCommandBuffer(mCmdBuf);

    VkSubmitInfo si = {};
    si.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    si.commandBufferCount = 1;
    si.pCommandBuffers = &mCmdBuf;
    mPfn->QueueSubmit(mQueue, 1, &si, mFence);
    mPfn->WaitForFences(mDevice, 1, &mFence, VK_TRUE, UINT64_MAX);
    mPfn->ResetFences(mDevice, 1, &mFence);

    /* Descriptor set — bind input buffer, scratch image, params buffer. */
    VkDescriptorBufferInfo inInfo = {};
    inInfo.buffer = mInBuf;    inInfo.range = mInSize;
    VkDescriptorImageInfo imgInfo = {};
    imgInfo.imageView = mScratchView;
    imgInfo.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
    VkDescriptorBufferInfo paramInfo = {};
    paramInfo.buffer = mParamBuf; paramInfo.range = sizeof(IspParams);

    VkWriteDescriptorSet writes[3] = {};
    writes[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[0].dstSet = mDescSet; writes[0].dstBinding = 0;
    writes[0].descriptorCount = 1;
    writes[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[0].pBufferInfo = &inInfo;

    writes[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[1].dstSet = mDescSet; writes[1].dstBinding = 1;
    writes[1].descriptorCount = 1;
    writes[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
    writes[1].pImageInfo = &imgInfo;

    writes[2].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[2].dstSet = mDescSet; writes[2].dstBinding = 2;
    writes[2].descriptorCount = 1;
    writes[2].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[2].pBufferInfo = &paramInfo;

    mPfn->UpdateDescriptorSets(mDevice, 3, writes, 0, NULL);

    return true;
}

/* --- main processing paths --- */

bool VulkanIspPipeline::process(const uint8_t *src, uint8_t *dst,
                                 unsigned width, unsigned height,
                                 uint32_t pixFmt) {
    /* CPU-fallback path (zero-copy disabled or gralloc ineligible) — synchronous.
     * The old deferred-readback 1-frame overlap was removed: zero-copy is now
     * the primary path and this one is rare; the added complexity bought us
     * nothing in the fallback. */
    return processSync(src, dst, width, height, pixFmt);
}

bool VulkanIspPipeline::processSync(const uint8_t *src, uint8_t *dst,
                                     unsigned width, unsigned height,
                                     uint32_t pixFmt) {
    if (!mReady) return false;

    bool is16 = (pixFmt == V4L2_PIX_FMT_SRGGB10 || pixFmt == V4L2_PIX_FMT_SGRBG10 ||
                 pixFmt == V4L2_PIX_FMT_SGBRG10 || pixFmt == V4L2_PIX_FMT_SBGGR10);
    if (!ensureBuffers(width, height, is16))
        return false;

    if (mPrevPending) {
        mPfn->WaitForFences(mDevice, 1, &mFence, VK_TRUE, UINT64_MAX);
        mPfn->ResetFences(mDevice, 1, &mFence);
        mPrevPending = false;
    }

    memcpy(mInMap, src, mInSize);
    IspParams params;
    fillParams(&params, width, height, is16, pixFmt);
    memcpy(mParamMap, &params, sizeof(IspParams));

    VkMappedMemoryRange flushRanges[2] = {};
    flushRanges[0].sType = VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE;
    flushRanges[0].memory = mInMem;
    flushRanges[0].size = VK_WHOLE_SIZE;
    flushRanges[1].sType = VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE;
    flushRanges[1].memory = mParamMem;
    flushRanges[1].size = VK_WHOLE_SIZE;
    mPfn->FlushMappedMemoryRanges(mDevice, 2, flushRanges);

    recordAndSubmit(width, height, mFence, true);
    mPfn->WaitForFences(mDevice, 1, &mFence, VK_TRUE, UINT64_MAX);
    mPfn->ResetFences(mDevice, 1, &mFence);

    VkMappedMemoryRange outRange = {};
    outRange.sType = VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE;
    outRange.memory = mOutMem;
    outRange.size = VK_WHOLE_SIZE;
    mPfn->InvalidateMappedMemoryRanges(mDevice, 1, &outRange);
    memcpy(dst, mOutMap, mOutSize);

    return true;
}

void VulkanIspPipeline::prewarm(unsigned width, unsigned height, uint32_t pixFmt) {
    if (!mReady) return;

    bool is16 = (pixFmt == V4L2_PIX_FMT_SRGGB10 || pixFmt == V4L2_PIX_FMT_SGRBG10 ||
                 pixFmt == V4L2_PIX_FMT_SGBRG10 || pixFmt == V4L2_PIX_FMT_SBGGR10);
    if (!ensureBuffers(width, height, is16))
        return;

    IspParams params = {};
    params.width = width;
    params.height = height;
    params.is16bit = is16 ? 1 : 0;
    params.wbR = 256; params.wbG = 256; params.wbB = 256;
    params.ccm[0] = 1024; params.ccm[4] = 1024; params.ccm[8] = 1024;
    memcpy(mParamMap, &params, sizeof(IspParams));

    VkMappedMemoryRange paramRange = {};
    paramRange.sType = VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE;
    paramRange.memory = mParamMem;
    paramRange.size = VK_WHOLE_SIZE;
    mPfn->FlushMappedMemoryRanges(mDevice, 1, &paramRange);

    int64_t start = nowMs();
    recordAndSubmit(width, height, mFence, false);
    mPfn->WaitForFences(mDevice, 1, &mFence, VK_TRUE, UINT64_MAX);
    mPfn->ResetFences(mDevice, 1, &mFence);
    ALOGD("Vulkan ISP prewarm %ux%u is16=%d: %lldms",
          width, height, is16 ? 1 : 0, nowMs() - start);
}

bool VulkanIspPipeline::processToGralloc(const uint8_t *src, void *nativeBuffer,
                                          unsigned width, unsigned height,
                                          uint32_t pixFmt,
                                          int acquireFence, int *releaseFence) {
    (void)acquireFence;
    *releaseFence = -1;

    if (!mReady || !nativeBuffer || !mNativeBufferAvail)
        return false;

    bool is16 = (pixFmt == V4L2_PIX_FMT_SRGGB10 || pixFmt == V4L2_PIX_FMT_SGRBG10 ||
                 pixFmt == V4L2_PIX_FMT_SGBRG10 || pixFmt == V4L2_PIX_FMT_SBGGR10);
    if (!ensureBuffers(width, height, is16))
        return false;

    ANativeWindowBuffer *anwb = (ANativeWindowBuffer *)nativeBuffer;
    GrallocEntry *entry = NULL;
    if (!getOrCreateGrallocImage(anwb, width, height, &entry))
        return false;

    int64_t t0 = nowMs();

    /* Drain any previous async work — we reuse mFence / mCmdBuf / mInMap. */
    if (mPrevPending) {
        mPfn->WaitForFences(mDevice, 1, &mFence, VK_TRUE, UINT64_MAX);
        mPfn->ResetFences(mDevice, 1, &mFence);
        mPrevPending = false;
    }

    memcpy(mInMap, src, mInSize);
    IspParams params;
    fillParams(&params, width, height, is16, pixFmt);
    memcpy(mParamMap, &params, sizeof(IspParams));

    VkMappedMemoryRange flushRanges[2] = {};
    flushRanges[0].sType = VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE;
    flushRanges[0].memory = mInMem;
    flushRanges[0].size = VK_WHOLE_SIZE;
    flushRanges[1].sType = VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE;
    flushRanges[1].memory = mParamMem;
    flushRanges[1].size = VK_WHOLE_SIZE;
    mPfn->FlushMappedMemoryRanges(mDevice, 2, flushRanges);

    int64_t t1 = nowMs();

    /* Record: dispatch (writes to mScratchImg via binding=1 set in ensureBuffers)
     * → barrier → vkCmdCopyImage(scratch → gralloc). Driver's tiler translates
     * pitchlinear scratch to whatever layout the gralloc handle dictates. */
    VkCommandBufferBeginInfo bi = {};
    bi.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    mPfn->ResetCommandBuffer(mCmdBuf, 0);
    mPfn->BeginCommandBuffer(mCmdBuf, &bi);

    /* Compute: Bayer → mScratchImg (via binding=1 storage image, set once in
     * ensureBuffers). */
    mPfn->CmdBindPipeline(mCmdBuf, VK_PIPELINE_BIND_POINT_COMPUTE, mPipeline);
    mPfn->CmdBindDescriptorSets(mCmdBuf, VK_PIPELINE_BIND_POINT_COMPUTE,
                                mPipeLayout, 0, 1, &mDescSet, 0, NULL);
    mPfn->CmdDispatch(mCmdBuf, (width + 7) / 8, (height + 7) / 8, 1);

    /* scratch SHADER_WRITE → SHADER_READ for the fragment pass. */
    VkImageMemoryBarrier scratchB = {};
    scratchB.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    scratchB.oldLayout = VK_IMAGE_LAYOUT_GENERAL;
    scratchB.newLayout = VK_IMAGE_LAYOUT_GENERAL;
    scratchB.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
    scratchB.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
    scratchB.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    scratchB.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    scratchB.image = mScratchImg;
    scratchB.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    scratchB.subresourceRange.levelCount = 1;
    scratchB.subresourceRange.layerCount = 1;
    mPfn->CmdPipelineBarrier(mCmdBuf,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
        VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT,
        0, 0, NULL, 0, NULL, 1, &scratchB);

    /* Fragment pass: full-screen triangle samples scratch, driver's ROP
     * rasterizes into gralloc's blocklinear layout correctly. */
    VkRenderPassBeginInfo rpbi = {};
    rpbi.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    rpbi.renderPass = mRenderPass;
    rpbi.framebuffer = entry->framebuffer;
    rpbi.renderArea.extent.width  = width;
    rpbi.renderArea.extent.height = height;

    mPfn->CmdBeginRenderPass(mCmdBuf, &rpbi, VK_SUBPASS_CONTENTS_INLINE);
    mPfn->CmdBindPipeline(mCmdBuf, VK_PIPELINE_BIND_POINT_GRAPHICS, mBlitPipeline);
    mPfn->CmdBindDescriptorSets(mCmdBuf, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                mPipeLayout, 0, 1, &mDescSet, 0, NULL);

    VkViewport vp = {};
    vp.width  = (float)width;
    vp.height = (float)height;
    vp.minDepth = 0.0f;
    vp.maxDepth = 1.0f;
    mPfn->CmdSetViewport(mCmdBuf, 0, 1, &vp);

    VkRect2D sc = {};
    sc.extent.width  = width;
    sc.extent.height = height;
    mPfn->CmdSetScissor(mCmdBuf, 0, 1, &sc);

    mPfn->CmdDraw(mCmdBuf, 3, 1, 0, 0);
    mPfn->CmdEndRenderPass(mCmdBuf);

    mPfn->EndCommandBuffer(mCmdBuf);
    entry->layoutReady = true;

    VkSubmitInfo si = {};
    si.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    si.commandBufferCount = 1;
    si.pCommandBuffers = &mCmdBuf;
    mPfn->QueueSubmit(mQueue, 1, &si, mFence);

    /* Ask the driver to emit a sync_fence fd that signals once the main submit
     * above (plus any internal release barrier) completes. The framework owns
     * this fd and waits on it before the compositor samples the buffer. */
    if (mPfn->QueueSignalReleaseImageANDROID) {
        int fd = -1;
        VkResult qr = mPfn->QueueSignalReleaseImageANDROID(mQueue, 0, NULL,
                                                            entry->image, &fd);
        if (qr == VK_SUCCESS) *releaseFence = fd;
        else ALOGW("vkQueueSignalReleaseImageANDROID failed: %d", (int)qr);
    }

    mPrevPending = true;

    int64_t t2 = nowMs();

    updateAwb(src, width, height, is16, pixFmt);

    ALOGD("VK gralloc: upload=%lld submit=%lldms", t1 - t0, t2 - t1);
    return true;
}

/* --- cleanup --- */

void VulkanIspPipeline::destroy() {
    if (mDevice != VK_NULL_HANDLE && mPfn && mPfn->DeviceWaitIdle) {
        mPfn->DeviceWaitIdle(mDevice);

        if (mInMap)    { mPfn->UnmapMemory(mDevice, mInMem);    mInMap = NULL; }
        if (mOutMap)   { mPfn->UnmapMemory(mDevice, mOutMem);   mOutMap = NULL; }
        if (mParamMap) { mPfn->UnmapMemory(mDevice, mParamMem); mParamMap = NULL; }

        clearGrallocImages();

        if (mScratchView) { mPfn->DestroyImageView(mDevice, mScratchView, NULL); mScratchView = VK_NULL_HANDLE; }
        if (mScratchImg)  { mPfn->DestroyImage(mDevice, mScratchImg, NULL);      mScratchImg = VK_NULL_HANDLE; }
        if (mScratchMem)  { mPfn->FreeMemory(mDevice, mScratchMem, NULL);        mScratchMem = VK_NULL_HANDLE; }

        destroyBuffer(mInBuf,    mInMem);
        destroyBuffer(mOutBuf,   mOutMem);
        destroyBuffer(mParamBuf, mParamMem);
        mInBuf = VK_NULL_HANDLE;  mInMem = VK_NULL_HANDLE;
        mOutBuf = VK_NULL_HANDLE; mOutMem = VK_NULL_HANDLE;
        mParamBuf = VK_NULL_HANDLE; mParamMem = VK_NULL_HANDLE;

        if (mFence)        { mPfn->DestroyFence(mDevice, mFence, NULL);                     mFence = VK_NULL_HANDLE; }
        if (mCmdPool)      { mPfn->DestroyCommandPool(mDevice, mCmdPool, NULL);             mCmdPool = VK_NULL_HANDLE; }
        if (mDescPool)     { mPfn->DestroyDescriptorPool(mDevice, mDescPool, NULL);         mDescPool = VK_NULL_HANDLE; }
        if (mBlitPipeline) { mPfn->DestroyPipeline(mDevice, mBlitPipeline, NULL);           mBlitPipeline = VK_NULL_HANDLE; }
        if (mPipeline)     { mPfn->DestroyPipeline(mDevice, mPipeline, NULL);               mPipeline = VK_NULL_HANDLE; }
        if (mRenderPass)   { mPfn->DestroyRenderPass(mDevice, mRenderPass, NULL);           mRenderPass = VK_NULL_HANDLE; }
        if (mPipeLayout)   { mPfn->DestroyPipelineLayout(mDevice, mPipeLayout, NULL);       mPipeLayout = VK_NULL_HANDLE; }
        if (mDescLayout)   { mPfn->DestroyDescriptorSetLayout(mDevice, mDescLayout, NULL);  mDescLayout = VK_NULL_HANDLE; }
        if (mFragShader)   { mPfn->DestroyShaderModule(mDevice, mFragShader, NULL);         mFragShader = VK_NULL_HANDLE; }
        if (mVertShader)   { mPfn->DestroyShaderModule(mDevice, mVertShader, NULL);         mVertShader = VK_NULL_HANDLE; }
        if (mShader)       { mPfn->DestroyShaderModule(mDevice, mShader, NULL);             mShader = VK_NULL_HANDLE; }
        if (mPfn->DestroyDevice) mPfn->DestroyDevice(mDevice, NULL);
        mDevice = VK_NULL_HANDLE;
    }
    if (mInstance != VK_NULL_HANDLE && mPfn && mPfn->DestroyInstance) {
        mPfn->DestroyInstance(mInstance, NULL);
        mInstance = VK_NULL_HANDLE;
    }

    delete mPfn;    mPfn = NULL;
    delete mLoader; mLoader = NULL;

    mReady = false;
    mParamsTemplateReady = false;
    mInSize = 0; mOutSize = 0;
    mBufWidth = 0; mBufHeight = 0;
    mPrevPending = false;
    mNativeBufferAvail = false;
}

void VulkanIspPipeline::clearGrallocImages() {
    if (!mPfn || mDevice == VK_NULL_HANDLE) {
        mGrallocImages.clear();
        return;
    }
    for (auto &kv : mGrallocImages) {
        if (kv.second.framebuffer) mPfn->DestroyFramebuffer(mDevice, kv.second.framebuffer, NULL);
        if (kv.second.view)        mPfn->DestroyImageView(mDevice, kv.second.view, NULL);
        if (kv.second.image)       mPfn->DestroyImage(mDevice, kv.second.image, NULL);
    }
    mGrallocImages.clear();
}

bool VulkanIspPipeline::getOrCreateGrallocImage(ANativeWindowBuffer *anwb,
                                                 unsigned width, unsigned height,
                                                 GrallocEntry **outEntry) {
    auto it = mGrallocImages.find(anwb->handle);
    if (it != mGrallocImages.end()) {
        *outEntry = &it->second;
        return true;
    }

    VkNativeBufferANDROID nbInfo = {};
    nbInfo.sType  = VK_STRUCTURE_TYPE_NATIVE_BUFFER_ANDROID;
    nbInfo.handle = anwb->handle;
    nbInfo.stride = anwb->stride;
    nbInfo.format = anwb->format;
    nbInfo.usage  = anwb->usage;

    VkImageCreateInfo ici = {};
    ici.sType         = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    ici.pNext         = &nbInfo;
    ici.imageType     = VK_IMAGE_TYPE_2D;
    ici.format        = VK_FORMAT_R8G8B8A8_UNORM;
    ici.extent.width  = width;
    ici.extent.height = height;
    ici.extent.depth  = 1;
    ici.mipLevels     = 1;
    ici.arrayLayers   = 1;
    ici.samples       = VK_SAMPLE_COUNT_1_BIT;
    ici.tiling        = VK_IMAGE_TILING_OPTIMAL;
    /* Write through the ROP path: only blocklinear-aware Tegra write surface. */
    ici.usage         = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
    ici.sharingMode   = VK_SHARING_MODE_EXCLUSIVE;
    ici.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

    GrallocEntry entry = {};
    VkResult r = mPfn->CreateImage(mDevice, &ici, NULL, &entry.image);
    if (r != VK_SUCCESS) {
        ALOGE("gralloc vkCreateImage failed: %d", (int)r);
        return false;
    }

    VkImageViewCreateInfo vci = {};
    vci.sType    = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    vci.image    = entry.image;
    vci.viewType = VK_IMAGE_VIEW_TYPE_2D;
    vci.format   = VK_FORMAT_R8G8B8A8_UNORM;
    vci.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    vci.subresourceRange.levelCount = 1;
    vci.subresourceRange.layerCount = 1;
    r = mPfn->CreateImageView(mDevice, &vci, NULL, &entry.view);
    if (r != VK_SUCCESS) {
        ALOGE("gralloc vkCreateImageView failed: %d", (int)r);
        mPfn->DestroyImage(mDevice, entry.image, NULL);
        return false;
    }

    VkFramebufferCreateInfo fci = {};
    fci.sType           = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
    fci.renderPass      = mRenderPass;
    fci.attachmentCount = 1;
    fci.pAttachments    = &entry.view;
    fci.width           = width;
    fci.height          = height;
    fci.layers          = 1;
    r = mPfn->CreateFramebuffer(mDevice, &fci, NULL, &entry.framebuffer);
    if (r != VK_SUCCESS) {
        ALOGE("gralloc vkCreateFramebuffer failed: %d", (int)r);
        mPfn->DestroyImageView(mDevice, entry.view, NULL);
        mPfn->DestroyImage(mDevice, entry.image, NULL);
        return false;
    }

    entry.layoutReady = false;
    auto res = mGrallocImages.emplace(anwb->handle, entry);
    *outEntry = &res.first->second;
    ALOGD("gralloc image cached: handle=%p image=%p view=%p (size=%zu)",
          anwb->handle, entry.image, entry.view, mGrallocImages.size());
    return true;
}

}; /* namespace android */
