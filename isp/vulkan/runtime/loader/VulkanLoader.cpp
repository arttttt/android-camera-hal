#define LOG_TAG "Cam-VkLoader"
#include <utils/Log.h>

#include "VulkanLoader.h"
#include "VulkanPfn.h"
#include "HalHmiVulkanLoader.h"
#include "SystemVulkanLoader.h"

namespace android {

/* Pick by what the driver actually offers, not by OS version.
 *
 * The split used to be "API 26+ means the system loader", on the assumption
 * that a modern libvulkan brings the gralloc interop the ISP needs. On this
 * device it does not, and the deciding factor turned out to be the driver
 * rather than the platform: through the system loader Android reports
 *
 *   External memory device ext: KHR_external_memory=1 fd=1 dma_buf=0 NV=0
 *   VK_ANDROID_native_buffer: absent
 *
 * VK_EXT_external_memory_dma_buf is missing, so a gralloc dmabuf cannot be
 * imported (only memory Vulkan itself exported, as OPAQUE_FD), and
 * VK_ANDROID_native_buffer is deliberately hidden by the loader -- it is
 * reserved for the loader's own swapchain implementation and never surfaces
 * to a client. Both are visible when the HAL module is opened directly,
 * which is exactly what the zero-copy path needs, so try that first.
 *
 * The system loader stays as the fallback: it is the correct path wherever
 * the driver does expose the interop extensions, and it is what a device
 * without a Tegra HAL module would use. */
VulkanLoader *createVulkanLoader() {
    VulkanLoader *hmi = new HalHmiVulkanLoader();
    if (hmi->load())
        return hmi;

    ALOGD("HAL-module Vulkan unavailable, falling back to the system loader");
    delete hmi;
    return new SystemVulkanLoader();
}

void VulkanLoader::loadInstancePfns(VkInstance instance, VulkanPfn *pfn) const {
    PFN_vkGetInstanceProcAddr gipa = getInstanceProcAddr();

    pfn->GetInstanceProcAddr                  = gipa;
    pfn->CreateInstance                       = getCreateInstance();
    pfn->EnumerateInstanceExtensionProperties = getEnumerateInstanceExtensionProperties();

    #define INST(name) \
        pfn->name = (PFN_vk##name)gipa(instance, "vk" #name)

    INST(DestroyInstance);
    INST(EnumeratePhysicalDevices);
    INST(GetPhysicalDeviceMemoryProperties);
    INST(GetPhysicalDeviceQueueFamilyProperties);
    INST(GetPhysicalDeviceProperties);
    INST(EnumerateDeviceExtensionProperties);
    INST(CreateDevice);
    INST(GetDeviceProcAddr);

    #undef INST
}

void VulkanLoader::loadDevicePfns(VkDevice device, VulkanPfn *pfn) const {
    PFN_vkGetDeviceProcAddr gdpa = pfn->GetDeviceProcAddr;

    #define DEV(name) \
        pfn->name = (PFN_vk##name)gdpa(device, "vk" #name)

    DEV(DestroyDevice);
    DEV(GetDeviceQueue);
    DEV(DeviceWaitIdle);

    DEV(QueueSubmit);
    DEV(QueueWaitIdle);

    DEV(AllocateMemory);
    DEV(FreeMemory);
    DEV(MapMemory);
    DEV(UnmapMemory);
    DEV(FlushMappedMemoryRanges);
    DEV(InvalidateMappedMemoryRanges);

    DEV(CreateBuffer);
    DEV(DestroyBuffer);
    DEV(GetBufferMemoryRequirements);
    DEV(BindBufferMemory);

    DEV(CreateImage);
    DEV(DestroyImage);
    DEV(GetImageMemoryRequirements);
    DEV(BindImageMemory);
    DEV(CreateImageView);
    DEV(DestroyImageView);
    DEV(CreateSampler);
    DEV(DestroySampler);

    DEV(CreateFence);
    DEV(DestroyFence);
    DEV(WaitForFences);
    DEV(ResetFences);

    DEV(CreateSemaphore);
    DEV(DestroySemaphore);

    DEV(CreateShaderModule);
    DEV(DestroyShaderModule);
    DEV(CreateDescriptorSetLayout);
    DEV(DestroyDescriptorSetLayout);
    DEV(CreatePipelineLayout);
    DEV(DestroyPipelineLayout);
    DEV(CreateComputePipelines);
    DEV(DestroyPipeline);

    DEV(CreateDescriptorPool);
    DEV(DestroyDescriptorPool);
    DEV(AllocateDescriptorSets);
    DEV(UpdateDescriptorSets);

    DEV(CreateRenderPass);
    DEV(DestroyRenderPass);
    DEV(CreateFramebuffer);
    DEV(DestroyFramebuffer);
    DEV(CreateGraphicsPipelines);

    DEV(CreateCommandPool);
    DEV(DestroyCommandPool);
    DEV(AllocateCommandBuffers);
    DEV(BeginCommandBuffer);
    DEV(EndCommandBuffer);
    DEV(ResetCommandBuffer);

    DEV(CmdBindPipeline);
    DEV(CmdBindDescriptorSets);
    DEV(CmdPushConstants);
    DEV(CmdDispatch);
    DEV(CmdPipelineBarrier);
    DEV(CmdCopyImageToBuffer);
    DEV(CmdCopyImage);
    DEV(CmdCopyBufferToImage);
    DEV(CmdFillBuffer);

    DEV(CreateQueryPool);
    DEV(DestroyQueryPool);
    DEV(CmdResetQueryPool);
    DEV(CmdWriteTimestamp);
    DEV(GetQueryPoolResults);

    DEV(CmdBeginRenderPass);
    DEV(CmdEndRenderPass);
    DEV(CmdDraw);
    DEV(CmdSetViewport);
    DEV(CmdSetScissor);

    /* VK_ANDROID_native_buffer — only present when extension enabled on device */
    DEV(AcquireImageANDROID);
    DEV(QueueSignalReleaseImageANDROID);

    /* VK_KHR_external_memory_fd */
    DEV(GetMemoryFdPropertiesKHR);
    DEV(GetMemoryFdKHR);

    /* VK_KHR_external_fence_fd */
    DEV(GetFenceFdKHR);

    /* VK_KHR_external_semaphore_fd */
    DEV(ImportSemaphoreFdKHR);

    #undef DEV
}

} /* namespace android */
