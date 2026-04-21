#ifndef VULKAN_STATS_ENCODER_H
#define VULKAN_STATS_ENCODER_H

#include <stdint.h>
#include <stddef.h>

#define VK_USE_PLATFORM_ANDROID_KHR
#include <vulkan/vulkan.h>

#include "ipa/IpaStats.h"

namespace android {

class VulkanDeviceState;

/* GPU 3A-statistics reducer. Samples a scratch RGBA image owned by
 * the caller and writes a single IpaStats layout (128-bin histogram
 * + 16x16 patch-mean RGB + 16x16 Tenengrad sharpness) into its own
 * host-visible VkBuffer. Caller supplies the command buffer, scratch
 * view + sampler, and is responsible for the scratch-side
 * SHADER_WRITE -> SHADER_READ barrier before recordDispatch().
 *
 * recordDispatch() zero-fills the output buffer before the compute
 * dispatch (histogram uses atomicAdd, so it must start at zero) and
 * records a SHADER_WRITE -> HOST_READ buffer barrier after the
 * dispatch so the CPU can read immediately on fence signal.
 *
 * Dispatch shape is fixed: 16 x 16 workgroups (one per patch) of
 * 256 threads each. Resolution is passed as a push constant — the
 * shader derives per-patch pixel extents from imgW / imgH and so
 * supports any input size that fits the scratch image. */
class VulkanStatsEncoder {
public:
    explicit VulkanStatsEncoder(VulkanDeviceState &dev);
    ~VulkanStatsEncoder();

    bool init();
    void destroy();

    /* Allocate the IpaStats-shaped output buffer and map it for host
     * read. No-op if already allocated. */
    bool ensureBuffers();

    /* Bind the scratch image + sampler the compute shader will read.
     * Call once after ensureBuffers() and again whenever the scratch
     * view handle changes (owner resized the scratch image). */
    void bindScratchInput(VkImageView scratchView, VkSampler sampler);

    /* Append zero-fill + dispatch + HOST_READ buffer barrier to the
     * caller's command buffer. Caller must have already placed the
     * scratch SHADER_WRITE -> SHADER_READ barrier upstream. Does not
     * submit. */
    void recordDispatch(VkCommandBuffer cb,
                         unsigned width, unsigned height);

    /* CPU-mapped stats pointer. Valid after init() + ensureBuffers();
     * data is defined only once the submit's fence has signalled and
     * invalidateForCpu() has been called. */
    const IpaStats *mappedStats() const { return reinterpret_cast<const IpaStats *>(bufMap); }
    size_t          bufferSize() const  { return bufSize; }

    /* Invalidate the CPU cache range so reads after GPU submit see
     * fresh data. No-op on HOST_COHERENT memory. */
    void invalidateForCpu();

private:
    bool createShader();
    bool createDescriptorLayout();
    bool createPipeline();
    bool allocateDescriptor();
    void writeDescriptors();
    void releaseOutputBuffer();

    VulkanDeviceState &dev;

    VkShaderModule        shader;
    VkDescriptorSetLayout descLayout;
    VkPipelineLayout      pipeLayout;
    VkPipeline            pipeline;
    VkDescriptorPool      descPool;
    VkDescriptorSet       descSet;

    VkBuffer       buf;
    VkDeviceMemory bufMem;
    void          *bufMap;
    size_t         bufSize;

    /* Borrowed handles — owner keeps them alive. */
    VkImageView scratchView;
    VkSampler   scratchSampler;
};

} /* namespace android */

#endif /* VULKAN_STATS_ENCODER_H */
