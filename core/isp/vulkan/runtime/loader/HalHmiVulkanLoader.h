#ifndef HAL_HMI_VULKAN_LOADER_H
#define HAL_HMI_VULKAN_LOADER_H

#include "VulkanLoader.h"

namespace android {

/* Android 7.x Vulkan bootstrap that bypasses libvulkan.so.
 *
 * libvulkan.so on API 25 filters HAL-only extensions (notably
 * VK_ANDROID_native_buffer) from app-level callers. This loader opens
 * the Vulkan HAL shim (vulkan.tegra.so) directly via dlopen, retrieves
 * its HMI hw_module_t, opens the 'vk0' device, and exposes the
 * driver's native Instance-level entry points. The returned PFNs come
 * from the driver's dispatch table so HAL extensions are visible
 * through them. */
class HalHmiVulkanLoader : public VulkanLoader {
public:
    HalHmiVulkanLoader();
    ~HalHmiVulkanLoader() override;

    bool load() override;

    PFN_vkGetInstanceProcAddr
        getInstanceProcAddr() const override;
    PFN_vkCreateInstance
        getCreateInstance() const override;
    PFN_vkEnumerateInstanceExtensionProperties
        getEnumerateInstanceExtensionProperties() const override;

    const char *name() const override { return "HalHmi"; }

private:
    void *mDso;         /* dlopen handle for vulkan.tegra.so */
    void *mDevice;      /* hwvulkan_device_t *, type-erased to hide layout */
};

}; /* namespace android */

#endif /* HAL_HMI_VULKAN_LOADER_H */
