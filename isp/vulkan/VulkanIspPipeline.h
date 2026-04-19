#ifndef VULKAN_ISP_PIPELINE_H
#define VULKAN_ISP_PIPELINE_H

#include "IspPipeline.h"
#include "IspParams.h"
#include "runtime/VulkanDeviceState.h"
#include "io/VulkanInputRing.h"
#include "io/VulkanGrallocCache.h"

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

    bool process(const uint8_t *src, uint8_t *dst,
                 unsigned width, unsigned height,
                 uint32_t pixFmt) override;

    bool processSync(const uint8_t *src, uint8_t *dst,
                      unsigned width, unsigned height,
                      uint32_t pixFmt,
                      int srcInputSlot = -1) override;

    void prewarm(unsigned width, unsigned height, uint32_t pixFmt) override;

    bool processToGralloc(const uint8_t *src, void *nativeBuffer,
                           unsigned width, unsigned height,
                           uint32_t pixFmt,
                           int acquireFence, int *releaseFence,
                           int srcInputSlot = -1) override;

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

    /* Zero-copy gralloc path: record compute → memory barrier → render
     * pass blit of mScratchImg into the entry's framebuffer, no final
     * submit. Caller follows with submitWithReleaseFence(). */
    void recordGrallocBlit(VulkanGrallocCache::Entry *entry,
                            unsigned width, unsigned height);

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
};

}; /* namespace android */

#endif // VULKAN_ISP_PIPELINE_H
