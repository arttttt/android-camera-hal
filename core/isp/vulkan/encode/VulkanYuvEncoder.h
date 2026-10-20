#ifndef VULKAN_YUV_ENCODER_H
#define VULKAN_YUV_ENCODER_H

#include <stdint.h>
#include <stddef.h>

#define VK_USE_PLATFORM_ANDROID_KHR
#include <vulkan/vulkan.h>

namespace android {

class VulkanDeviceState;

/* GPU RGBA → NV12 converter. Samples a scratch RGBA image owned by the
 * caller and writes a single NV12 buffer (Y plane followed by
 * interleaved UV plane) into its own host-visible VkBuffer. Caller
 * supplies the command buffer, scratch view + sampler, and is
 * responsible for all synchronisation (scratch WRITE→READ barrier,
 * submit, fence wait).
 *
 * BT.601 limited range is hardcoded in the shader. NV12 out; any other
 * 420 layout (YV12, NV21, I420) is a pure byte repack on CPU outside
 * this class. */
class VulkanYuvEncoder {
public:
    explicit VulkanYuvEncoder(VulkanDeviceState &dev);
    ~VulkanYuvEncoder();

    bool init();
    void destroy();

    /* (Re)allocate the output buffer for a (width, height) image. No-op
     * when dimensions are unchanged. width must be divisible by 4 and
     * height by 2 (shader is a 4×2 block write). */
    bool ensureBuffers(unsigned width, unsigned height);

    /* Bind the scratch image + sampler the compute shader will sample.
     * Call once after ensureBuffers() and whenever the scratch view
     * handle changes (e.g. the owner's scratch image was resized). */
    void bindScratchInput(VkImageView scratchView, VkSampler sampler);

    /* Append an RGBA→NV12 dispatch to the caller's command buffer.
     * The caller must ensure the scratch image is in VK_IMAGE_LAYOUT_
     * GENERAL and a SHADER_WRITE→SHADER_READ barrier has been recorded.
     * Does not submit. */
    void recordDispatch(VkCommandBuffer cb, unsigned width, unsigned height);

    /* CPU-mapped NV12 base pointer. Y at [0, w*h); UV interleaved at
     * [w*h, w*h*3/2). Valid after init() + ensureBuffers(). */
    const uint8_t *mappedBuffer() const { return (const uint8_t *)mBufMap; }
    size_t         bufferSize()  const { return mBufSize; }

    /* Invalidate the CPU cache range so reads after GPU submit see
     * fresh data. No-op if memory is HOST_COHERENT. */
    void invalidateForCpu();

private:
    bool createShader();
    bool createDescriptorLayout();
    bool createPipeline();
    bool allocateDescriptor();
    void releaseOutputBuffer();
    void writeDescriptors();

    VulkanDeviceState &mDev;

    VkShaderModule        mShader;
    VkDescriptorSetLayout mDescLayout;
    VkPipelineLayout      mPipeLayout;
    VkPipeline            mPipeline;
    VkDescriptorPool      mDescPool;
    VkDescriptorSet       mDescSet;

    VkBuffer       mBuf;
    VkDeviceMemory mBufMem;
    void          *mBufMap;
    size_t         mBufSize;
    unsigned       mWidth, mHeight;

    /* borrowed — owner keeps these alive */
    VkImageView mScratchView;
    VkSampler   mScratchSampler;
};

}; /* namespace android */

#endif /* VULKAN_YUV_ENCODER_H */
