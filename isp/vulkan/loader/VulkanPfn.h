#ifndef VULKAN_PFN_H
#define VULKAN_PFN_H

#define VK_USE_PLATFORM_ANDROID_KHR
#include <vulkan/vulkan.h>

/* VK_ANDROID_native_buffer PFN typedefs — not exposed in android-24 NDK vulkan.h */
#ifndef VK_ANDROID_NATIVE_BUFFER_PFN
#define VK_ANDROID_NATIVE_BUFFER_PFN
typedef VkResult (VKAPI_PTR *PFN_vkAcquireImageANDROID)(
    VkDevice device, VkImage image, int nativeFenceFd,
    VkSemaphore semaphore, VkFence fence);
typedef VkResult (VKAPI_PTR *PFN_vkQueueSignalReleaseImageANDROID)(
    VkQueue queue, uint32_t waitSemaphoreCount,
    const VkSemaphore *pWaitSemaphores, VkImage image, int *pNativeFenceFd);
#endif

/* VK_KHR_external_memory + VK_KHR_external_memory_fd — also absent in android-24 */
#ifndef VK_KHR_EXTERNAL_MEMORY_FD_FALLBACK
#define VK_KHR_EXTERNAL_MEMORY_FD_FALLBACK
typedef enum VkExternalMemoryHandleTypeFlagBitsKHR {
    VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT_KHR         = 0x00000001,
    VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_WIN32_BIT_KHR      = 0x00000002,
    VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_WIN32_KMT_BIT_KHR  = 0x00000004,
    VK_EXTERNAL_MEMORY_HANDLE_TYPE_D3D11_TEXTURE_BIT_KHR     = 0x00000008,
    VK_EXTERNAL_MEMORY_HANDLE_TYPE_D3D11_TEXTURE_KMT_BIT_KHR = 0x00000010,
    VK_EXTERNAL_MEMORY_HANDLE_TYPE_D3D12_HEAP_BIT_KHR        = 0x00000020,
    VK_EXTERNAL_MEMORY_HANDLE_TYPE_D3D12_RESOURCE_BIT_KHR    = 0x00000040,
    VK_EXTERNAL_MEMORY_HANDLE_TYPE_DMA_BUF_BIT_EXT           = 0x00000200,
} VkExternalMemoryHandleTypeFlagBitsKHR;

#define VK_STRUCTURE_TYPE_IMPORT_MEMORY_FD_INFO_KHR  ((VkStructureType)1000074000)
#define VK_STRUCTURE_TYPE_MEMORY_FD_PROPERTIES_KHR   ((VkStructureType)1000074001)

typedef struct VkImportMemoryFdInfoKHR {
    VkStructureType                       sType;
    const void                           *pNext;
    VkExternalMemoryHandleTypeFlagBitsKHR handleType;
    int                                   fd;
} VkImportMemoryFdInfoKHR;

typedef struct VkMemoryFdPropertiesKHR {
    VkStructureType sType;
    void           *pNext;
    uint32_t        memoryTypeBits;
} VkMemoryFdPropertiesKHR;

typedef VkResult (VKAPI_PTR *PFN_vkGetMemoryFdPropertiesKHR)(
    VkDevice device, VkExternalMemoryHandleTypeFlagBitsKHR handleType,
    int fd, VkMemoryFdPropertiesKHR *pMemoryFdProperties);
#endif

namespace android {

/* Dispatch table of Vulkan function pointers populated by VulkanLoader.
 * Consumers call Vulkan through this struct instead of linked vk* symbols. */
struct VulkanPfn {
    /* Bootstrap (populated from VulkanLoader getters) */
    PFN_vkGetInstanceProcAddr                       GetInstanceProcAddr;
    PFN_vkCreateInstance                            CreateInstance;
    PFN_vkEnumerateInstanceExtensionProperties      EnumerateInstanceExtensionProperties;

    /* Instance-level (populated by VulkanLoader::loadInstancePfns) */
    PFN_vkDestroyInstance                           DestroyInstance;
    PFN_vkEnumeratePhysicalDevices                  EnumeratePhysicalDevices;
    PFN_vkGetPhysicalDeviceMemoryProperties         GetPhysicalDeviceMemoryProperties;
    PFN_vkGetPhysicalDeviceQueueFamilyProperties    GetPhysicalDeviceQueueFamilyProperties;
    PFN_vkEnumerateDeviceExtensionProperties        EnumerateDeviceExtensionProperties;
    PFN_vkCreateDevice                              CreateDevice;
    PFN_vkGetDeviceProcAddr                         GetDeviceProcAddr;

    /* Device-level (populated by VulkanLoader::loadDevicePfns) */
    PFN_vkDestroyDevice                             DestroyDevice;
    PFN_vkGetDeviceQueue                            GetDeviceQueue;
    PFN_vkDeviceWaitIdle                            DeviceWaitIdle;

    PFN_vkQueueSubmit                               QueueSubmit;
    PFN_vkQueueWaitIdle                             QueueWaitIdle;

    PFN_vkAllocateMemory                            AllocateMemory;
    PFN_vkFreeMemory                                FreeMemory;
    PFN_vkMapMemory                                 MapMemory;
    PFN_vkUnmapMemory                               UnmapMemory;
    PFN_vkFlushMappedMemoryRanges                   FlushMappedMemoryRanges;
    PFN_vkInvalidateMappedMemoryRanges              InvalidateMappedMemoryRanges;

    PFN_vkCreateBuffer                              CreateBuffer;
    PFN_vkDestroyBuffer                             DestroyBuffer;
    PFN_vkGetBufferMemoryRequirements               GetBufferMemoryRequirements;
    PFN_vkBindBufferMemory                          BindBufferMemory;

    PFN_vkCreateImage                               CreateImage;
    PFN_vkDestroyImage                              DestroyImage;
    PFN_vkGetImageMemoryRequirements                GetImageMemoryRequirements;
    PFN_vkBindImageMemory                           BindImageMemory;
    PFN_vkCreateImageView                           CreateImageView;
    PFN_vkDestroyImageView                          DestroyImageView;

    PFN_vkCreateFence                               CreateFence;
    PFN_vkDestroyFence                              DestroyFence;
    PFN_vkWaitForFences                             WaitForFences;
    PFN_vkResetFences                               ResetFences;

    PFN_vkCreateShaderModule                        CreateShaderModule;
    PFN_vkDestroyShaderModule                       DestroyShaderModule;
    PFN_vkCreateDescriptorSetLayout                 CreateDescriptorSetLayout;
    PFN_vkDestroyDescriptorSetLayout                DestroyDescriptorSetLayout;
    PFN_vkCreatePipelineLayout                      CreatePipelineLayout;
    PFN_vkDestroyPipelineLayout                     DestroyPipelineLayout;
    PFN_vkCreateComputePipelines                    CreateComputePipelines;
    PFN_vkDestroyPipeline                           DestroyPipeline;

    PFN_vkCreateDescriptorPool                      CreateDescriptorPool;
    PFN_vkDestroyDescriptorPool                     DestroyDescriptorPool;
    PFN_vkAllocateDescriptorSets                    AllocateDescriptorSets;
    PFN_vkUpdateDescriptorSets                      UpdateDescriptorSets;

    PFN_vkCreateCommandPool                         CreateCommandPool;
    PFN_vkDestroyCommandPool                        DestroyCommandPool;
    PFN_vkAllocateCommandBuffers                    AllocateCommandBuffers;
    PFN_vkBeginCommandBuffer                        BeginCommandBuffer;
    PFN_vkEndCommandBuffer                          EndCommandBuffer;
    PFN_vkResetCommandBuffer                        ResetCommandBuffer;

    PFN_vkCmdBindPipeline                           CmdBindPipeline;
    PFN_vkCmdBindDescriptorSets                     CmdBindDescriptorSets;
    PFN_vkCmdDispatch                               CmdDispatch;
    PFN_vkCmdPipelineBarrier                        CmdPipelineBarrier;
    PFN_vkCmdCopyImageToBuffer                      CmdCopyImageToBuffer;

    /* VK_ANDROID_native_buffer (HAL-only extension, gralloc zero-copy) */
    PFN_vkAcquireImageANDROID                       AcquireImageANDROID;
    PFN_vkQueueSignalReleaseImageANDROID            QueueSignalReleaseImageANDROID;

    /* VK_KHR_external_memory_fd — for importing gralloc dma-buf fd as VkDeviceMemory */
    PFN_vkGetMemoryFdPropertiesKHR                  GetMemoryFdPropertiesKHR;
};

}; /* namespace android */

#endif /* VULKAN_PFN_H */
