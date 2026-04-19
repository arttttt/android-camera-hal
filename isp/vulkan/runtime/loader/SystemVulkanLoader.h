#ifndef SYSTEM_VULKAN_LOADER_H
#define SYSTEM_VULKAN_LOADER_H

#include "VulkanLoader.h"

namespace android {

/* Vulkan bootstrap via the standard libvulkan.so loader.
 *
 * Intended for Android 8+ / API 26+, where the modern libvulkan exposes
 * VK_ANDROID_external_memory_android_hardware_buffer and other zero-copy
 * paths natively — no HAL-layer bypass required.
 *
 * Not implemented yet. This project currently targets Android 7.1.2
 * only; when newer-OS support lands, this class will dlopen libvulkan.so
 * and hand out its canonical entry points. */
class SystemVulkanLoader : public VulkanLoader {
public:
    SystemVulkanLoader() {}
    ~SystemVulkanLoader() override {}

    bool load() override;

    PFN_vkGetInstanceProcAddr
        getInstanceProcAddr() const override { return nullptr; }
    PFN_vkCreateInstance
        getCreateInstance() const override { return nullptr; }
    PFN_vkEnumerateInstanceExtensionProperties
        getEnumerateInstanceExtensionProperties() const override { return nullptr; }

    const char *name() const override { return "System"; }
};

}; /* namespace android */

#endif /* SYSTEM_VULKAN_LOADER_H */
