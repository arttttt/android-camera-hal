#ifndef VULKAN_LOADER_H
#define VULKAN_LOADER_H

#define VK_USE_PLATFORM_ANDROID_KHR
#include <vulkan/vulkan.h>

namespace android {

struct VulkanPfn;

/* Platform bootstrap for Vulkan. Caller asks the loader for the three
 * Instance-level entry points it needs to create a VkInstance, then
 * delegates population of the full PFN dispatch table to loadInstancePfns
 * and loadDevicePfns. */
class VulkanLoader {
public:
    virtual ~VulkanLoader() {}

    virtual bool load() = 0;

    virtual PFN_vkGetInstanceProcAddr
        getInstanceProcAddr() const = 0;
    virtual PFN_vkCreateInstance
        getCreateInstance() const = 0;
    virtual PFN_vkEnumerateInstanceExtensionProperties
        getEnumerateInstanceExtensionProperties() const = 0;

    virtual const char *name() const = 0;

    /* Fill instance-level entries of pfn using the loader's
     * vkGetInstanceProcAddr. Caller must have created instance already. */
    void loadInstancePfns(VkInstance instance, VulkanPfn *pfn) const;

    /* Fill device-level entries of pfn using pfn->GetDeviceProcAddr
     * (populated by loadInstancePfns). Caller must have created device. */
    void loadDevicePfns(VkDevice device, VulkanPfn *pfn) const;
};

VulkanLoader *createVulkanLoader();

}; /* namespace android */

#endif /* VULKAN_LOADER_H */
