#ifndef VULKAN_DEVICE_STATE_H
#define VULKAN_DEVICE_STATE_H

#include <stdint.h>

#define VK_USE_PLATFORM_ANDROID_KHR
#include <vulkan/vulkan.h>

namespace android {

class VulkanLoader;
struct VulkanPfn;

/* Owns the Vulkan runtime — loader, instance, physical/logical device,
 * compute queue, and the function-pointer dispatch table. One consumer
 * (VulkanIspPipeline) composes this and creates its own command pool,
 * descriptor sets and pipelines on top. */
class VulkanDeviceState {
public:
    VulkanDeviceState();
    ~VulkanDeviceState();

    bool init();
    void destroy();

    bool isReady() const { return mDevice != VK_NULL_HANDLE; }

    VkInstance       instance()    const { return mInstance; }
    VkPhysicalDevice physDev()     const { return mPhysDev; }
    VkDevice         device()      const { return mDevice; }
    VkQueue          queue()       const { return mQueue; }
    uint32_t         queueFamily() const { return mQueueFamily; }
    VulkanPfn       *pfn()         const { return mPfn; }

    /* VK_ANDROID_native_buffer is filtered out of the app-side libvulkan
     * on Android 7, but the HAL-variant loader exposes it. Gralloc
     * zero-copy output depends on it. */
    bool nativeBufferAvailable() const { return mNativeBufferAvail; }

    /* Find a memory type index matching the filter bitmask and property
     * flags, or UINT32_MAX when no match exists. */
    uint32_t findMemoryType(uint32_t filter, VkMemoryPropertyFlags props) const;

    /* Allocate a buffer + backing memory and bind them. When exportable
     * is true the memory is allocated with VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT_KHR
     * so its fd can be pulled out via vkGetMemoryFdKHR and handed to
     * V4L2 in DMABUF mode. Returns false and leaves *buf / *mem untouched
     * on failure. */
    bool createBuffer(VkBuffer *buf, VkDeviceMemory *mem,
                      VkDeviceSize size, VkBufferUsageFlags usage,
                      bool exportable = false) const;
    void destroyBuffer(VkBuffer buf, VkDeviceMemory mem) const;

private:
    VulkanLoader    *mLoader;
    VulkanPfn       *mPfn;
    VkInstance       mInstance;
    VkPhysicalDevice mPhysDev;
    VkDevice         mDevice;
    VkQueue          mQueue;
    uint32_t         mQueueFamily;
    bool             mNativeBufferAvail;
};

}; /* namespace android */

#endif /* VULKAN_DEVICE_STATE_H */
