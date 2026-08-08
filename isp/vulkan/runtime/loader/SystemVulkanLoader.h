#ifndef SYSTEM_VULKAN_LOADER_H
#define SYSTEM_VULKAN_LOADER_H

#include "VulkanLoader.h"

namespace android {

/* Vulkan bootstrap via the standard libvulkan.so loader.
 *
 * Used on Android 8+ / API 26+, where libvulkan exposes what the ISP needs --
 * VK_ANDROID_external_memory_android_hardware_buffer and the rest of the
 * zero-copy path -- on its own. Reaching past it into the driver module, the
 * way HalHmiVulkanLoader has to on 7.1.2, is not merely unnecessary there but
 * wrong: from O on, the system loader owns layer setup and the driver handle,
 * and a second, private handle to the same driver is not something the
 * implementation expects to exist. */
class SystemVulkanLoader : public VulkanLoader {
public:
    SystemVulkanLoader();
    ~SystemVulkanLoader() override;

    bool load() override;

    PFN_vkGetInstanceProcAddr
        getInstanceProcAddr() const override { return mGetInstanceProcAddr; }
    PFN_vkCreateInstance
        getCreateInstance() const override { return mCreateInstance; }
    PFN_vkEnumerateInstanceExtensionProperties
        getEnumerateInstanceExtensionProperties() const override {
            return mEnumerateInstanceExtensionProperties;
        }

    const char *name() const override { return "System"; }

private:
    void *mDso;
    PFN_vkGetInstanceProcAddr                  mGetInstanceProcAddr;
    PFN_vkCreateInstance                       mCreateInstance;
    PFN_vkEnumerateInstanceExtensionProperties mEnumerateInstanceExtensionProperties;
};

}; /* namespace android */

#endif /* SYSTEM_VULKAN_LOADER_H */
