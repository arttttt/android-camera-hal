#define LOG_TAG "Cam-VulkanDeviceState"
#include <utils/Log.h>

#include <string.h>

#include "VulkanDeviceState.h"
#include "loader/VulkanLoader.h"
#include "loader/VulkanPfn.h"

#ifndef VK_API_VERSION
#define VK_API_VERSION VK_MAKE_VERSION(1, 0, 0)
#endif

namespace android {

VulkanDeviceState::VulkanDeviceState()
    : mLoader(NULL)
    , mPfn(NULL)
    , mInstance(VK_NULL_HANDLE)
    , mPhysDev(VK_NULL_HANDLE)
    , mDevice(VK_NULL_HANDLE)
    , mQueue(VK_NULL_HANDLE)
    , mQueueFamily(0)
    , mNativeBufferAvail(false) {}

VulkanDeviceState::~VulkanDeviceState() { destroy(); }

bool VulkanDeviceState::init() {
    if (mDevice != VK_NULL_HANDLE) return true;

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

    bool hasGpdp2 = false, hasExtMemCaps = false, hasNvExtMemCaps = false;
    bool hasExtFenceCaps = false;
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
            if (!strcmp(ex[i].extensionName, "VK_KHR_external_fence_capabilities"))
                hasExtFenceCaps = true;
        }
        delete[] ex;
    }

    const char *iexts[8];
    uint32_t iec = 0;
    if (hasGpdp2)       iexts[iec++] = "VK_KHR_get_physical_device_properties2";
    if (hasExtMemCaps)  iexts[iec++] = "VK_KHR_external_memory_capabilities";
    if (hasNvExtMemCaps)iexts[iec++] = "VK_NV_external_memory_capabilities";
    if (hasExtFenceCaps)iexts[iec++] = "VK_KHR_external_fence_capabilities";

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

    uint32_t devCount = 1;
    if (mPfn->EnumeratePhysicalDevices(mInstance, &devCount, &mPhysDev) != VK_SUCCESS
            || devCount == 0) {
        ALOGE("No Vulkan physical device");
        destroy();
        return false;
    }

    bool hasNvGlsl = false;
    bool hasKhrExtMem = false, hasKhrExtMemFd = false;
    bool hasExtExtMemDmaBuf = false, hasNvExtMem = false;
    bool hasKhrExtFence = false, hasKhrExtFenceFd = false;
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
            if (!strcmp(exts[i].extensionName, "VK_KHR_external_fence"))
                hasKhrExtFence = true;
            if (!strcmp(exts[i].extensionName, "VK_KHR_external_fence_fd"))
                hasKhrExtFenceFd = true;
        }
        delete[] exts;
    }
    ALOGD("External memory device ext: KHR_external_memory=%d fd=%d dma_buf=%d NV=%d",
          hasKhrExtMem, hasKhrExtMemFd, hasExtExtMemDmaBuf, hasNvExtMem);
    ALOGD("External fence device ext: KHR_external_fence=%d fd=%d",
          hasKhrExtFence, hasKhrExtFenceFd);
    if (!hasNvGlsl) {
        ALOGE("VK_NV_glsl_shader not supported");
        destroy();
        return false;
    }
    ALOGD("VK_ANDROID_native_buffer: %s", mNativeBufferAvail ? "AVAILABLE" : "absent");

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

    float qPriority = 1.0f;
    VkDeviceQueueCreateInfo qci = {};
    qci.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
    qci.queueFamilyIndex = mQueueFamily;
    qci.queueCount = 1;
    qci.pQueuePriorities = &qPriority;

    const char *enabledExts[10];
    uint32_t enabledExtCount = 0;
    enabledExts[enabledExtCount++] = "VK_NV_glsl_shader";
    if (mNativeBufferAvail)   enabledExts[enabledExtCount++] = "VK_ANDROID_native_buffer";
    if (hasKhrExtMem)         enabledExts[enabledExtCount++] = "VK_KHR_external_memory";
    if (hasKhrExtMemFd)       enabledExts[enabledExtCount++] = "VK_KHR_external_memory_fd";
    if (hasExtExtMemDmaBuf)   enabledExts[enabledExtCount++] = "VK_EXT_external_memory_dma_buf";
    if (hasNvExtMem)          enabledExts[enabledExtCount++] = "VK_NV_external_memory";
    if (hasKhrExtFence)       enabledExts[enabledExtCount++] = "VK_KHR_external_fence";
    if (hasKhrExtFenceFd)     enabledExts[enabledExtCount++] = "VK_KHR_external_fence_fd";

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
    return true;
}

void VulkanDeviceState::destroy() {
    if (mDevice != VK_NULL_HANDLE && mPfn && mPfn->DestroyDevice) {
        mPfn->DestroyDevice(mDevice, NULL);
        mDevice = VK_NULL_HANDLE;
    }
    if (mInstance != VK_NULL_HANDLE && mPfn && mPfn->DestroyInstance) {
        mPfn->DestroyInstance(mInstance, NULL);
        mInstance = VK_NULL_HANDLE;
    }

    delete mPfn;    mPfn = NULL;
    delete mLoader; mLoader = NULL;

    mPhysDev = VK_NULL_HANDLE;
    mQueue = VK_NULL_HANDLE;
    mQueueFamily = 0;
    mNativeBufferAvail = false;
}

uint32_t VulkanDeviceState::findMemoryType(uint32_t filter, VkMemoryPropertyFlags props) const {
    VkPhysicalDeviceMemoryProperties memProps;
    mPfn->GetPhysicalDeviceMemoryProperties(mPhysDev, &memProps);
    for (uint32_t i = 0; i < memProps.memoryTypeCount; i++) {
        if ((filter & (1 << i)) && (memProps.memoryTypes[i].propertyFlags & props) == props)
            return i;
    }
    return UINT32_MAX;
}

bool VulkanDeviceState::createBuffer(VkBuffer *buf, VkDeviceMemory *mem,
                                      VkDeviceSize size, VkBufferUsageFlags usage,
                                      bool exportable) const {
    VkExternalMemoryBufferCreateInfoKHR emb = {};
    emb.sType = VK_STRUCTURE_TYPE_EXTERNAL_MEMORY_BUFFER_CREATE_INFO_KHR;
    emb.handleTypes = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT_KHR;

    VkBufferCreateInfo ci = {};
    ci.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    ci.pNext = exportable ? &emb : NULL;
    ci.size = size;
    ci.usage = usage;
    ci.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    if (mPfn->CreateBuffer(mDevice, &ci, NULL, buf) != VK_SUCCESS)
        return false;

    VkMemoryRequirements req;
    mPfn->GetBufferMemoryRequirements(mDevice, *buf, &req);

    VkExportMemoryAllocateInfoKHR emi = {};
    emi.sType = VK_STRUCTURE_TYPE_EXPORT_MEMORY_ALLOCATE_INFO_KHR;
    emi.handleTypes = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT_KHR;

    VkMemoryAllocateInfo ai = {};
    ai.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    ai.pNext = exportable ? &emi : NULL;
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

void VulkanDeviceState::destroyBuffer(VkBuffer buf, VkDeviceMemory mem) const {
    if (buf != VK_NULL_HANDLE) mPfn->DestroyBuffer(mDevice, buf, NULL);
    if (mem != VK_NULL_HANDLE) mPfn->FreeMemory(mDevice, mem, NULL);
}

}; /* namespace android */
