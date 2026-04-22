#ifndef VULKAN_ISP_PIPELINE_H
#define VULKAN_ISP_PIPELINE_H

#include "IspPipeline.h"
#include "IspParams.h"
#include "runtime/VulkanDeviceState.h"
#include "io/VulkanInputRing.h"
#include "io/VulkanGrallocCache.h"
#include "encode/VulkanStatsEncoder.h"
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
                           int acquireFence,
                           int *releaseFence, int *submitFence,
                           int srcInputSlot,
                           const CropRect &crop) override;

    int    inputBufferCount() const override { return mInputRing.slotCount(); }
    size_t inputBufferSize()  const override { return mInputRing.slotSize(); }
    int    exportInputBufferFd(int idx) override { return mInputRing.exportFd(idx); }
    void   waitForPreviousFrame() override;
    void   onSessionClose() override;

    void            invalidateStats() override { mStatsEncoder.invalidateForCpu(); }
    const IpaStats *mappedStats()    const override { return mStatsEncoder.mappedStats(); }

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

    /* Round-robin slot pick. If the chosen slot is still in flight on
     * the GPU, block on WaitForFences and reset before returning.
     * In steady state (processToGralloc spacing ≥ 1 frame) the slot's
     * fence has already signalled so the wait is a no-op — the ring
     * turns into a pure overlap-enabler. */
    int acquireSlot();

    /* Write `p` into the slot's param buffer and flush the memory range
     * so the GPU sees the new parameters on the next dispatch. */
    void uploadParams(int slot, const IspParams &p);

    /* Rebind slot's descriptor binding=0 to the input ring slot — cheap
     * (one vkUpdateDescriptorSets call) compared to re-allocating the
     * set. */
    void rebindInputDescriptor(int slot, int inputSlot);

    /* Record the slot's cmd buffer with pipeline+descriptor bind and a
     * single compute dispatch sized for (width, height); optionally
     * append an image→buffer copy of mScratchImg into mOutBuf so CPU
     * can read the result via mOutMap. Submits on mFence[slot]. */
    void recordAndSubmit(int slot, unsigned width, unsigned height,
                          bool copyToOutBuf);

    /* Record the slot's cmd buffer with demosaic → scratch → yuv-encode
     * compute chain and submit on mFence[slot]. The yuv encoder writes
     * its own mapped NV12 buffer; caller reads it via
     * mYuvEncoder.mappedBuffer() after fence wait. */
    void recordDemosaicAndYuvEncode(int slot, unsigned width, unsigned height);

    /* Zero-copy gralloc path: record compute → memory barrier → render
     * pass blit of mScratchImg into the entry's framebuffer, no final
     * submit. Caller follows with submitWithReleaseFence().
     * srcW/H:  scratch image size (matches the capture resolution the
     *          compute demosaic writes).
     * dstW/H:  framebuffer / gralloc output size.
     * crop:    sub-region of the scratch to sample, in source coords.
     *          Identity blit passes {0, 0, srcW, srcH} with dst == src. */
    void recordGrallocBlit(int slot, VulkanGrallocCache::Entry *entry,
                            unsigned srcW, unsigned srcH,
                            unsigned dstW, unsigned dstH,
                            const CropRect &crop);

    /* Submit slot's cmd buffer on mFence[slot] and ask the driver for
     * a sync_fence fd that signals once the blit completes — the
     * framework waits on it before compositing. Writes -1 to
     * *releaseFence on failure. */
    void submitWithReleaseFence(int slot, VulkanGrallocCache::Entry *entry,
                                 int *releaseFence);

    /* Per-submit GPU resources are held in a round-robin ring so the
     * CPU side of frame N+1 (cmd-buffer record + descriptor update +
     * vkQueueSubmit) can overlap with the GPU side of frame N. The
     * single-queue Tegra K1 serialises execution between submits
     * anyway, so one scratch image shared across slots is safe — the
     * ring buys CPU↔GPU overlap, not parallel GPU execution. */
    static constexpr size_t SLOT_COUNT = 4;

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
    VkDescriptorSet mDescSet[SLOT_COUNT];
    VkSampler mScratchSampler;   /* linear/clamp-to-edge, bound at binding=3 */
    VkCommandPool mCmdPool;
    VkCommandBuffer mCmdBuf[SLOT_COUNT];

    VkBuffer       mOutBuf;
    VkDeviceMemory mOutMem;
    size_t         mOutSize;
    void          *mOutMap;

    /* One param buffer per slot — the compute shader dispatches of two
     * in-flight submits must see independent IspParams content. */
    VkBuffer       mParamBuf[SLOT_COUNT];
    VkDeviceMemory mParamMem[SLOT_COUNT];
    void          *mParamMap[SLOT_COUNT];

    /* Shader output image for CPU-readback paths; copied to mOutBuf after dispatch. */
    VkImage        mScratchImg;
    VkDeviceMemory mScratchMem;
    VkImageView    mScratchView;

    VkFence mFence[SLOT_COUNT];
    /* sync_fd exported from the slot's fence via vkGetFenceFdKHR; -1
     * when the slot has no pending async submit. vkGetFenceFdKHR with
     * SYNC_FD implicitly resets the fence, so the reused fence goes
     * back to unsignalled state ready for the next submit — our
     * "slot still in flight?" signal is the poll-readability of this
     * fd, not the fence. Only the async gralloc path populates it;
     * sync paths (processToCpu / Yuv / prewarm) drain in-line via
     * WaitForFences and leave the slot's sync_fd at -1. */
    int mSlotSyncFd[SLOT_COUNT];
    size_t mNextSlot;

    void fillParams(IspParams *p, unsigned w, unsigned h, bool is16, uint32_t pixFmt);
    void updateAwb(const uint8_t *raw, unsigned w, unsigned h, bool is16, uint32_t pixFmt);

    /* Precomputed sRGB-encode LUT (linear [0..255] → encoded [0..255]),
     * copied into every IspParams upload. Filled once in init(). */
    uint32_t mGammaLut[256];
    void buildGammaLut();

    VulkanGrallocCache mGrallocCache;
    VulkanYuvEncoder   mYuvEncoder;   /* lazy — buffers allocated on first YUV request */
    VulkanStatsEncoder mStatsEncoder; /* always on — buffer sized once at init() */

    /* Per-frame GPU-side timestamps for recordGrallocBlit phases.
     * 4 queries per slot: 0 = top of pipe, 1 = after demosaic,
     * 2 = after stats, 3 = after blit. Read in processToGralloc
     * right after WaitForFences, logged as PERF-GPU. Null when the
     * driver reports timestampValidBits == 0. */
    VkQueryPool        mTimeQuery;
    static constexpr size_t TIMESTAMPS_PER_SLOT = 4;

    /* Temporal subsample of the stats dispatch. 3A consumers update
     * at 10–15 Hz; at 22 fps preview every second stats frame is
     * dropped on the floor anyway. Running the compute only on every
     * Nth gralloc submit halves the GPU stats cost on average, and
     * the per-frame state carries over (the stats buffer holds the
     * previous dispatch's result until the next one lands). */
    static constexpr uint32_t STATS_INTERVAL = 2;
    uint32_t           mStatsFrameCounter;
};

}; /* namespace android */

#endif // VULKAN_ISP_PIPELINE_H
