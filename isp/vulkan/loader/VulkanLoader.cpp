#include <cutils/properties.h>

#include "VulkanLoader.h"
#include "VulkanPfn.h"
#include "HalHmiVulkanLoader.h"
#include "SystemVulkanLoader.h"

namespace android {

static const int API_OREO = 26;

VulkanLoader *createVulkanLoader() {
    int sdk = property_get_int32("ro.build.version.sdk", 0);
    if (sdk >= API_OREO)
        return new SystemVulkanLoader();
    return new HalHmiVulkanLoader();
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

    DEV(CreateFence);
    DEV(DestroyFence);
    DEV(WaitForFences);
    DEV(ResetFences);

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
    DEV(CmdDispatch);
    DEV(CmdPipelineBarrier);
    DEV(CmdCopyImageToBuffer);
    DEV(CmdCopyImage);
    DEV(CmdCopyBufferToImage);

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

    #undef DEV
}

} /* namespace android */
