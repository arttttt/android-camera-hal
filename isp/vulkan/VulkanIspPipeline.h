#ifndef VULKAN_ISP_PIPELINE_H
#define VULKAN_ISP_PIPELINE_H

#include "IspPipeline.h"
#include "IspParams.h"
#include "runtime/VulkanDeviceState.h"
#include "io/VulkanInputRing.h"
#include "io/VulkanGrallocCache.h"
#include "encode/VulkanYuvEncoder.h"

#define VK_USE_PLATFORM_ANDROID_KHR
#include <vulkan/vulkan.h>

struct ANativeWindowBuffer;

namespace android {

class VulkanIspPipeline : public IspPipeline {
public:
    VulkanIspPipeline();
    ~VulkanIspPipeline();

    bool init() override;
    void destroy() override;

    const uint8_t *processToCpu(const uint8_t *src,
                                 unsigned width, unsigned height,
                                 uint32_t pixFmt,
                                 int srcInputSlot) override;

    const uint8_t *processToYuv420(const uint8_t *src,
                                     unsigned width, unsigned height,
                                     uint32_t pixFmt,
                                     int srcInputSlot) override;

    void prewarm(unsigned width, unsigned height, uint32_t pixFmt) override;

    bool processToGralloc(const uint8_t *src, void *nativeBuffer,
                           unsigned srcW, unsigned srcH,
                           unsigned dstW, unsigned dstH,
                           uint32_t pixFmt,
                           int acquireFence, int *releaseFence,
                           int srcInputSlot,
                           const CropRect &crop) override;

    int    inputBufferCount() const override { return mInputRing.slotCount(); }
    size_t inputBufferSize()  const override { return mInputRing.slotSize(); }
    int    exportInputBufferFd(int idx) override { return mInputRing.exportFd(idx); }
    void   waitForPreviousFrame() override;

private:
    /* init() sub-steps. Each returns false on Vulkan failure; init() calls
     * destroy() on any false return so a partial state is never left behind. */
    bool compileShader(const char *glsl, VkShaderModule *out);
    bool createShaders();
    bool createDescriptorLayouts();
    bool createComputePipeline();
    bool createGraphicsPipeline();
    bool allocateDescriptorSet();
    bool createCommandObjects();

    bool ensureBuffers(unsigned width, unsigned height, bool is16bit);

    /* ensureBuffers sub-steps. */
    void releaseScratchResources();
    bool createOutBuffer(size_t size);
    bool createScratchImage(unsigned width, unsigned height);
    void writeStaticDescriptors();

    /* Wait + reset mFence when mPrevPending is set; no-op otherwise.
     * Used to sync with the async processToGralloc submit before the
     * next frame starts touching mCmdBuf / mDescSet / mParamMap. */
    void drainPendingFence();

    /* Write `p` into mParamMap and flush the memory range so the GPU
     * sees the new parameters on the next dispatch. */
    void uploadParams(const IspParams &p);

    /* Rebind descriptor binding=0 to the input ring slot — cheap (one
     * vkUpdateDescriptorSets call) compared to re-allocating the set. */
    void rebindInputDescriptor(int slot);

    /* Record mCmdBuf with pipeline+descriptor bind and a single compute
     * dispatch sized for (width, height); optionally append an image→buffer
     * copy of mScratchImg into mOutBuf so CPU can read the result via mOutMap.
     * Submits to mQueue signalling `fence` (VK_NULL_HANDLE = no fence). */
    void recordAndSubmit(unsigned width, unsigned height, VkFence fence,
                          bool copyToOutBuf);

    /* Record mCmdBuf with demosaic → scratch → yuv-encode compute chain
     * and submit. The yuv encoder writes its own mapped NV12 buffer; the
     * caller reads it via mYuvEncoder.mappedBuffer() after fence wait. */
    void recordDemosaicAndYuvEncode(unsigned width, unsigned height, VkFence fence);

    /* Zero-copy gralloc path: record compute → memory barrier → render
     * pass blit of mScratchImg into the entry's framebuffer, no final
     * submit. Caller follows with submitWithReleaseFence().
     * srcW/H:  scratch image size (matches the capture resolution the
     *          compute demosaic writes).
     * dstW/H:  framebuffer / gralloc output size.
     * crop:    sub-region of the scratch to sample, in source coords.
     *          Identity blit passes {0, 0, srcW, srcH} with dst == src. */
    void recordGrallocBlit(VulkanGrallocCache::Entry *entry,
                            unsigned srcW, unsigned srcH,
                            unsigned dstW, unsigned dstH,
                            const CropRect &crop);

    /* Submit mCmdBuf on mFence and ask the driver for a sync_fence fd
     * that signals once the blit completes — the framework waits on it
     * before compositing. Writes -1 to *releaseFence on failure. */
    void submitWithReleaseFence(VulkanGrallocCache::Entry *entry,
                                 int *releaseFence);

    VulkanDeviceState mDeviceState;
    VulkanInputRing   mInputRing;

    bool mReady;
    unsigned mBufWidth, mBufHeight;

    VkShaderModule mShader;
    VkShaderModule mVertShader;
    VkShaderModule mFragShader;
    VkDescriptorSetLayout mDescLayout;
    VkPipelineLayout mPipeLayout;
    VkPipeline mPipeline;          /* compute: Bayer → scratch image */
    VkPipeline mBlitPipeline;      /* graphics: scratch → gralloc via ROP */
    VkRenderPass mRenderPass;      /* 1 color attachment, RGBA8, DONT_CARE→STORE */
    VkDescriptorPool mDescPool;
    VkDescriptorSet mDescSet;
    VkSampler mScratchSampler;   /* linear/clamp-to-edge, bound at binding=3 */
    VkCommandPool mCmdPool;
    VkCommandBuffer mCmdBuf;

    VkBuffer mOutBuf, mParamBuf;
    VkDeviceMemory mOutMem, mParamMem;
    size_t mOutSize;
    void *mOutMap, *mParamMap;

    /* Shader output image for CPU-readback paths; copied to mOutBuf after dispatch. */
    VkImage        mScratchImg;
    VkDeviceMemory mScratchMem;
    VkImageView    mScratchView;

    VkFence mFence;
    /* Set by the async processToGralloc path to indicate mFence / mCmdBuf
     * are still in use by GPU. Drained at the start of the next call. */
    bool mPrevPending;

    void fillParams(IspParams *p, unsigned w, unsigned h, bool is16, uint32_t pixFmt);
    void updateAwb(const uint8_t *raw, unsigned w, unsigned h, bool is16, uint32_t pixFmt);

    VulkanGrallocCache mGrallocCache;
    VulkanYuvEncoder   mYuvEncoder;   /* lazy — buffers allocated on first YUV request */
};

}; /* namespace android */

#endif // VULKAN_ISP_PIPELINE_H
