#define LOG_TAG "Cam-VkLoader"
#include <utils/Log.h>
#include <dlfcn.h>

#include "HalHmiVulkanLoader.h"

namespace android {
namespace {

/* Inline minimal Android HAL structs — avoid NDK hardware header
 * dependency. Layouts must match hardware/libhardware/include/hardware/
 * hardware.h and hwvulkan.h on Android 7.1.2. */

struct hw_module_methods_inline {
    int (*open)(const void *module, const char *id, void **device);
};

struct hw_module_inline {
    uint32_t                     tag;
    uint16_t                     module_api_version;
    uint16_t                     hal_api_version;
    const char                  *id;
    const char                  *name;
    const char                  *author;
    hw_module_methods_inline    *methods;
    void                        *dso;
    uint32_t                     reserved[32 - 7];
};

struct hw_device_inline {
    uint32_t                     tag;
    uint32_t                     version;
    void                        *module;
    uint32_t                     reserved[12];
    int                         (*close)(void *device);
};

struct hwvulkan_device_inline {
    hw_device_inline                              common;
    PFN_vkEnumerateInstanceExtensionProperties    EnumerateInstanceExtensionProperties;
    PFN_vkCreateInstance                          CreateInstance;
    PFN_vkGetInstanceProcAddr                     GetInstanceProcAddr;
};

inline hwvulkan_device_inline *asDev(void *p) {
    return reinterpret_cast<hwvulkan_device_inline *>(p);
}

} /* anonymous namespace */

HalHmiVulkanLoader::HalHmiVulkanLoader()
    : mDso(nullptr), mDevice(nullptr) {}

HalHmiVulkanLoader::~HalHmiVulkanLoader() {
    if (mDevice) {
        hwvulkan_device_inline *dev = asDev(mDevice);
        if (dev->common.close) dev->common.close(dev);
        mDevice = nullptr;
    }
    if (mDso) { dlclose(mDso); mDso = nullptr; }
}

bool HalHmiVulkanLoader::load() {
    mDso = dlopen("/system/vendor/lib/hw/vulkan.tegra.so",
                  RTLD_NOW | RTLD_LOCAL);
    if (!mDso) {
        ALOGE("HalHmi: dlopen vulkan.tegra.so failed: %s", dlerror());
        return false;
    }

    hw_module_inline *hwMod =
        reinterpret_cast<hw_module_inline *>(dlsym(mDso, "HMI"));
    if (!hwMod) {
        ALOGE("HalHmi: dlsym HMI failed: %s", dlerror());
        return false;
    }
    if (!hwMod->methods || !hwMod->methods->open) {
        ALOGE("HalHmi: HMI has no open method");
        return false;
    }

    void *raw = nullptr;
    int r = hwMod->methods->open(hwMod, "vk0", &raw);
    if (r != 0 || !raw) {
        ALOGE("HalHmi: HMI open('vk0') failed: %d", r);
        return false;
    }
    mDevice = raw;

    hwvulkan_device_inline *dev = asDev(mDevice);
    if (!dev->CreateInstance ||
        !dev->EnumerateInstanceExtensionProperties ||
        !dev->GetInstanceProcAddr) {
        ALOGE("HalHmi: hwvulkan_device missing required entry points");
        return false;
    }

    ALOGD("HalHmi: loaded '%s' via HMI (CreateInstance=%p, GIPA=%p)",
          hwMod->name ? hwMod->name : "?",
          dev->CreateInstance,
          dev->GetInstanceProcAddr);
    return true;
}

PFN_vkGetInstanceProcAddr
HalHmiVulkanLoader::getInstanceProcAddr() const {
    return mDevice ? asDev(mDevice)->GetInstanceProcAddr : nullptr;
}

PFN_vkCreateInstance
HalHmiVulkanLoader::getCreateInstance() const {
    return mDevice ? asDev(mDevice)->CreateInstance : nullptr;
}

PFN_vkEnumerateInstanceExtensionProperties
HalHmiVulkanLoader::getEnumerateInstanceExtensionProperties() const {
    return mDevice ? asDev(mDevice)->EnumerateInstanceExtensionProperties
                   : nullptr;
}

} /* namespace android */
