#ifndef VULKAN_LOADER_H
#define VULKAN_LOADER_H

#define VK_USE_PLATFORM_ANDROID_KHR
#include <vulkan/vulkan.h>

namespace android {

/* Platform-specific Vulkan bootstrap.
 *
 * On Android 7.x / API 25, libvulkan.so filters HAL-only extensions
 * (notably VK_ANDROID_native_buffer, needed for zero-copy gralloc output)
 * from app-level callers. A HAL that wants them must bypass libvulkan and
 * talk to the Vulkan HAL module (vulkan.tegra.so on Tegra K1) directly.
 *
 * On Android 8+ / API 26+ the modern libvulkan plus
 * VK_ANDROID_external_memory_android_hardware_buffer give a cleaner
 * standard path — no bypass needed.
 *
 * VulkanLoader hides this choice: the caller asks for three bootstrap
 * entry points (vkCreateInstance, vkEnumerateInstanceExtensionProperties,
 * vkGetInstanceProcAddr) and proceeds as usual. All subsequent Vulkan
 * work happens through PFNs obtained via the returned
 * vkGetInstanceProcAddr / vkGetDeviceProcAddr. */
class VulkanLoader {
public:
    virtual ~VulkanLoader() {}

    /* Perform platform-specific bootstrap. Returns false if this loader
     * is not applicable on the current platform (caller should fall back
     * to another VulkanLoader implementation or give up). */
    virtual bool load() = 0;

    /* Bootstrap entry points. All three are non-null after a successful
     * load(). */
    virtual PFN_vkGetInstanceProcAddr
        getInstanceProcAddr() const = 0;
    virtual PFN_vkCreateInstance
        getCreateInstance() const = 0;
    virtual PFN_vkEnumerateInstanceExtensionProperties
        getEnumerateInstanceExtensionProperties() const = 0;

    /* Short tag for log messages. */
    virtual const char *name() const = 0;
};

/* Pick a VulkanLoader implementation appropriate for this build. Caller
 * owns the returned pointer. Never returns null (falls back to a no-op
 * stub that always returns false from load()). */
VulkanLoader *createVulkanLoader();

}; /* namespace android */

#endif /* VULKAN_LOADER_H */
