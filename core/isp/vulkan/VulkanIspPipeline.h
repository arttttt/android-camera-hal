#ifndef VULKAN_ISP_PIPELINE_H
#define VULKAN_ISP_PIPELINE_H

#include <atomic>
#include <vector>

#include "IspPipeline.h"
#include "IspParams.h"
#include "runtime/VkOwned.h"
#include "runtime/VulkanDeviceState.h"
#include "io/VulkanInputRing.h"
#include "io/VulkanGrallocCache.h"
#include "encode/VulkanYuvEncoder.h"

#define VK_USE_PLATFORM_ANDROID_KHR
#include <vulkan/vulkan.h>

#include <utils/RefBase.h>

struct ANativeWindowBuffer;

namespace android {

class GraphicBuffer;

class VulkanIspPipeline : public IspPipeline {
public:
    VulkanIspPipeline();
    ~VulkanIspPipeline();

    bool init() override;
    void destroy() override;

    void prewarm(unsigned width, unsigned height, uint32_t pixFmt) override;

    bool beginFrame(unsigned srcW, unsigned srcH, uint32_t pixFmt,
                     int srcInputSlot, float digitalGain) override;
    bool blitToGralloc(mocha::camera::NativeBuffer *buffer,
                        unsigned dstW, unsigned dstH,
                        const CropRect &crop,
                        int acquireFence,
                        int *releaseFenceOut) override;
    bool blitToYuv(mocha::camera::NativeBuffer *buffer,
                    unsigned dstW, unsigned dstH,
                    const CropRect &crop,
                    int acquireFence,
                    int *releaseFenceOut) override;
    bool blitToJpegCpu(const CropRect &crop, JpegSnapshot *out) override;
    void invalidateJpegSnapshot(const JpegSnapshot &snap) override;
    void releaseJpegSnapshot(const JpegSnapshot &snap) override;
    bool endFrame(int *submitFenceOut) override;

    const uint8_t *yuvHostBuffer() const override { return mYuvEncoder.mappedBuffer(); }
    void           invalidateYuvForCpu() override { mYuvEncoder.invalidateForCpu(); }

    int    inputBufferCount() const override { return mInputRing.slotCount(); }
    int    exportInputBufferFd(int idx) override { return mInputRing.exportFd(idx); }
    bool   deviceLost() const override { return mDeviceLost.load(); }

    /* Taken by the next fillParams; the 3A tick writes them while the
     * demosaic engine records. */
    void setWbGains(unsigned r, unsigned g, unsigned b) override {
        mWbR.store(r); mWbG.store(g); mWbB.store(b);
    }
    void setCcm(const int16_t *ccm) override { mCcm.store(ccm); }
    void setBlackLevel(uint32_t v) override  { mBlackLevel.store(v); }
    void setEnabled(bool en) override        { mEnabled.store(en); }
    void   onSessionClose() override;

    const void *bayerHost(int slot)   const override { return mInputRing.mapped(slot); }
    void        invalidateBayer(int slot) override   { mInputRing.invalidateFromGpu(slot); }

private:
    /* Block until the GPU work of the previous submit has completed. */
    void waitForPreviousFrame();

    std::atomic<unsigned>        mWbR{256}, mWbG{256}, mWbB{256};
    std::atomic<const int16_t *> mCcm{nullptr};
    std::atomic<uint32_t>        mBlackLevel{0};
    std::atomic<bool>            mEnabled{true};

    /* init() sub-steps. Each returns false on Vulkan failure; init() calls
     * destroy() on any false return so a partial state is never left behind. */
    bool compileShader(const char *glsl, VkOwned<VkShaderModule> *out);
    bool createShaders();
    bool createDescriptorLayouts();
    bool createComputePipeline();
    bool createGraphicsPipeline();
    bool allocateDescriptorSet();
    bool createCommandObjects();

    bool ensureBuffers(unsigned width, unsigned height, bool is16bit);

    /* ensureBuffers sub-steps. */
    void releaseScratchResources();
    bool createScratchImage(unsigned width, unsigned height);
    void writeStaticDescriptors();

    /* Round-robin slot pick. If the chosen slot is still in flight on
     * the GPU, block on its sync_fd via poll() before returning. In
     * steady state (frame spacing ≥ 1 frame_period) the slot's fence
     * has already signalled so the wait is a no-op — the ring buys
     * CPU↔GPU overlap. Also destroys any leftover acquire-fence
     * semaphores from the slot's previous submit. */
    int acquireSlot();     /* -1: the slot's fence timed out (GPU hung) */

    /* Waits for a slot's previous submit, at most kGpuFenceTimeoutMs.
     * On timeout the device is marked lost and false returned. */
    static constexpr int kGpuFenceTimeoutMs = 1000;
    /* One-off setup submits (scratch layout, prewarm) may include the
     * driver's first pipeline compile; give them more room. */
    static constexpr int kGpuSetupTimeoutMs = 5000;
    bool waitSlotFence(int slot);
    void waitFenceInline(int slot, int timeoutMs);

    /* Write `p` into the slot's param buffer and flush the memory range
     * so the GPU sees the new parameters on the next dispatch. */
    void uploadParams(int slot, const IspParams &p);

    /* Rebind slot's descriptor binding=0 to the input ring slot — cheap
     * (one vkUpdateDescriptorSets call) compared to re-allocating the
     * set. */
    void rebindInputDescriptor(int slot, int inputSlot);

    /* Record the slot's cmd buffer with pipeline+descriptor bind and a
     * single demosaic compute dispatch sized for (width, height); end
     * the cmd buffer and submit on mFence[slot]. Used by prewarm. */
    void recordAndSubmit(int slot, unsigned width, unsigned height);

    /* Open the slot's cmd buffer and record demosaic compute + scratch
     * write→read barrier. Buffer stays open for blit append calls. */
    bool recordDemosaicOpen(int slot, unsigned srcW, unsigned srcH);

    /* Result check for the calls a frame cannot do without. Logs a
     * failure; VK_ERROR_DEVICE_LOST also marks the device lost. Costs
     * a compare on the value the call returns anyway. */
    bool vkOk(VkResult r, const char *what);
    std::atomic<bool> mDeviceLost{false};

    /* Append a sampler-blit render pass (scratch → gralloc framebuffer)
     * to the open cmd buffer. */
    void recordRgbaBlitRenderPass(int slot, VulkanGrallocCache::Entry *entry,
                                    unsigned srcW, unsigned srcH,
                                    unsigned dstW, unsigned dstH,
                                    const CropRect &crop);

    /* Append a compute NV12 encode dispatch (scratch → mYuvEncoder buffer)
     * to the open cmd buffer. */
    void recordYuvEncodeDispatch(int slot, unsigned width, unsigned height,
                                 const CropRect &crop);

    /* Import a framework acquire_fence (sync_fd) as a binary VkSemaphore
     * with TEMPORARY semantic: subsequent submit waits on it; once the
     * wait fires the semaphore reverts to permanent (no-payload) state
     * and is safe to destroy after the submit's fence signals. */
    bool importAcquireSemaphore(int acquireFence, VkSemaphore *out);

    /* Per-submit GPU resources are held in a round-robin ring so the
     * CPU side of frame N+1 (cmd-buffer record + descriptor update +
     * vkQueueSubmit) can overlap with the GPU side of frame N. The
     * single-queue Tegra K1 serialises execution between submits
     * anyway, so one scratch image shared across slots is safe — the
     * ring buys CPU↔GPU overlap, not parallel GPU execution. */
    static constexpr size_t SLOT_COUNT = 4;
    static_assert(VulkanGrallocCache::kIdleFrames > SLOT_COUNT,
                  "an evicted gralloc entry must be past every submit in flight");

    /* JPEG snapshot ring depth — sized for the in-flight BLOB encode
     * pipeline, not the per-submit slot ring. The JPEG engine holds one
     * snapshot during the libjpeg encode (~150 ms at 1080p);
     * the demosaic engine can stage the next snapshot in parallel. Two
     * slots cover that producer/consumer overlap. Larger only matters
     * if the framework starts queuing multiple BLOB requests
     * back-to-back at >1 / encode-period — not how Camera3 actually
     * uses BLOB. Each slot is a host-mapped scratch-sized RGBA8 buffer
     * (~8 MB at 1080p) carved out of the shared nvmap pool, so depth
     * matters for memory pressure on Tegra K1. */
    static constexpr size_t JPEG_RING_DEPTH = 2;

    VulkanDeviceState mDeviceState;
    VulkanInputRing   mInputRing;

    bool mReady;
    unsigned mBufWidth, mBufHeight;

    VkOwned<VkShaderModule>        mShader;
    VkOwned<VkShaderModule>        mVertShader;
    VkOwned<VkShaderModule>        mFragShader;
    VkOwned<VkDescriptorSetLayout> mDescLayout;
    VkOwned<VkPipelineLayout>      mPipeLayout;
    VkOwned<VkPipeline>            mPipeline;       /* compute: Bayer → scratch image */
    VkOwned<VkPipeline>            mBlitPipeline;   /* graphics: scratch → gralloc via ROP */
    VkOwned<VkRenderPass>          mRenderPass;     /* 1 color attachment, RGBA8, DONT_CARE→STORE */
    VkOwned<VkDescriptorPool>      mDescPool;
    VkDescriptorSet mDescSet[SLOT_COUNT];
    VkOwned<VkSampler>             mScratchSampler; /* linear/clamp-to-edge, bound at binding=3 */
    VkOwned<VkCommandPool>         mCmdPool;
    VkCommandBuffer mCmdBuf[SLOT_COUNT];

    /* One param buffer per slot — the compute shader dispatches of two
     * in-flight submits must see independent IspParams content. */
    VkBuffer       mParamBuf[SLOT_COUNT];
    VkDeviceMemory mParamMem[SLOT_COUNT];
    void          *mParamMap[SLOT_COUNT];

    /* Shader output image — sampled by the produce-once blits and
     * read by vkCmdCopyImageToBuffer for the JPEG ring. Backed by a
     * gralloc-allocated RGBA buffer and imported through
     * VK_ANDROID_native_buffer (see createScratchImage). */
    sp<GraphicBuffer>      mScratchGb;
    VkOwned<VkImage>       mScratchImg;
    VkOwned<VkImageView>   mScratchView;

    VkOwned<VkFence> mFence[SLOT_COUNT];
    /* sync_fd exported from the slot's fence via vkGetFenceFdKHR; -1
     * when the slot has no pending async submit. vkGetFenceFdKHR with
     * SYNC_FD implicitly resets the fence, so the reused fence goes
     * back to unsignalled state ready for the next submit. The
     * produce-once endFrame populates this; prewarm drains in-line
     * via WaitForFences and leaves the slot's sync_fd at -1. */
    int mSlotSyncFd[SLOT_COUNT];
    size_t mNextSlot;

    void fillParams(IspParams *p, unsigned w, unsigned h, bool is16, uint32_t pixFmt,
                    float digitalGain);

    /* Precomputed sRGB-encode LUT (linear [0..255] → encoded [0..255]),
     * copied into every IspParams upload. Filled once in init(). */
    uint32_t mGammaLut[256];
    void buildGammaLut();

    VulkanGrallocCache mGrallocCache;
    VulkanYuvEncoder   mYuvEncoder;   /* lazy — buffers allocated on first YUV request */

    /* In-flight per-slot semaphores: the acquire fences imported with a
     * TEMPORARY payload during blitTo*, and the release semaphores the
     * submit signals for each gralloc output (endFrame). Held until the
     * slot is reused, then destroyed. One vector per slot -- slot reuse
     * implies the prior submit's fence has signalled (acquireSlot waits
     * on it), and with it the releases queued right behind it, so the
     * prior frame's semaphores have all been waited and are safe to
     * destroy. */
    std::vector<VkSemaphore> mSlotSemaphores[SLOT_COUNT];

    /* Ring of host-mapped RGBA8 buffers for blitToJpegCpu. Sized to the
     * current scratch dimensions; reallocated when ensureBuffers resizes
     * the scratch. A JPEG job holds a snapshot for the duration of one
     * libjpeg encode; blitToJpegCpu picks the first inUse=false slot.
     *
     * inUse is taken on the demosaic engine and given back by the JPEG
     * job (or the finish engine when the frame never reached the
     * encoder):
     * the release store orders the encoder's last read of the slot
     * before the next frame's copy into it. */
    struct JpegRingEntry {
        VkBuffer          buf;
        VkDeviceMemory    mem;
        void             *mapped;
        size_t            size;
        std::atomic<bool> inUse;
    };
    JpegRingEntry mJpegRing[JPEG_RING_DEPTH];
    unsigned      mJpegRingW, mJpegRingH;

    bool ensureJpegRing(unsigned w, unsigned h);
    void releaseJpegRing();

    /* Per-frame produce-once recording state. Active between beginFrame
     * and endFrame; reset on either path's failure. */
    struct PendingBlit {
        VulkanGrallocCache::Entry *entry;   /* RGBA only; null for YUV */
        int                       *releaseFenceOut;
    };
    struct FrameRecording {
        bool                       active = false;
        int                        slot = -1;
        unsigned                   srcW = 0;
        unsigned                   srcH = 0;
        std::vector<VkSemaphore>   waitSemaphores;
        std::vector<PendingBlit>   blits;

        void reset() {
            active = false;
            slot = -1;
            srcW = 0;
            srcH = 0;
            waitSemaphores.clear();
            blits.clear();
        }
    };
    FrameRecording mRec;
};

}; /* namespace android */

#endif // VULKAN_ISP_PIPELINE_H
