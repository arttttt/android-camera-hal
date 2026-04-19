#ifndef VULKAN_ISP_PIPELINE_H
#define VULKAN_ISP_PIPELINE_H

#include "IspPipeline.h"
#include "IspParams.h"
#include "VulkanDeviceState.h"
#include "VulkanInputRing.h"

#include <cutils/native_handle.h>
#include <unordered_map>

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
    bool ensureBuffers(unsigned width, unsigned height, bool is16bit);

    /* Record mCmdBuf with pipeline+descriptor bind and a single compute
     * dispatch sized for (width, height); optionally append an image→buffer
     * copy of mScratchImg into mOutBuf so CPU can read the result via mOutMap.
     * Submits to mQueue signalling `fence` (VK_NULL_HANDLE = no fence). */
    void recordAndSubmit(unsigned width, unsigned height, VkFence fence,
                          bool copyToOutBuf);

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

    /* Cached VkImage + view + framebuffer per gralloc buffer_handle_t for
     * zero-copy output. Gralloc owns the backing memory; we only track the
     * Vulkan wrappers. */
    struct GrallocEntry {
        VkImage image;
        VkImageView view;
        VkFramebuffer framebuffer;
        bool layoutReady;  /* UNDEFINED → COLOR_ATTACHMENT_OPTIMAL transitioned */
    };
    std::unordered_map<const native_handle_t *, GrallocEntry> mGrallocImages;

    bool getOrCreateGrallocImage(ANativeWindowBuffer *anwb,
                                  unsigned width, unsigned height,
                                  GrallocEntry **outEntry);
    void clearGrallocImages();
};

}; /* namespace android */

#endif // VULKAN_ISP_PIPELINE_H
