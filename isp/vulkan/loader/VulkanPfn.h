#ifndef VULKAN_PFN_H
#define VULKAN_PFN_H

#define VK_USE_PLATFORM_ANDROID_KHR
#include <vulkan/vulkan.h>

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
};

}; /* namespace android */

#endif /* VULKAN_PFN_H */
