#ifndef VULKAN_ISP_PIPELINE_H
#define VULKAN_ISP_PIPELINE_H

#include "IspPipeline.h"

#include <cutils/native_handle.h>
#include <unordered_map>

#define VK_USE_PLATFORM_ANDROID_KHR
#include <vulkan/vulkan.h>

struct ANativeWindowBuffer;

namespace android {

class VulkanLoader;
struct VulkanPfn;

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
                      uint32_t pixFmt) override;

    void prewarm(unsigned width, unsigned height, uint32_t pixFmt) override;

    bool processToGralloc(const uint8_t *src, void *nativeBuffer,
                           unsigned width, unsigned height,
                           uint32_t pixFmt,
                           int acquireFence, int *releaseFence) override;

    int    inputBufferCount() const override { return kInputBufferCount; }
    size_t inputBufferSize()  const override { return mInSize; }
    int    exportInputBufferFd(int idx) override;

    /* Ring depth for V4L2 ↔ Vulkan Bayer input hand-off. Four matches the
     * default V4L2 queue depth and gives the capture thread one buffer of
     * slack between DQBUF and GPU-consume. */
    static const int kInputBufferCount = 4;

private:
    bool createBuffer(VkBuffer *buf, VkDeviceMemory *mem,
                      VkDeviceSize size, VkBufferUsageFlags usage,
                      bool exportable = false);
    void destroyBuffer(VkBuffer buf, VkDeviceMemory mem);
    uint32_t findMemoryType(uint32_t filter, VkMemoryPropertyFlags props);

    /* Diagnostic: allocate a small exportable VkBuffer/VkDeviceMemory and try
     * vkGetMemoryFdKHR. Tells us whether the Tegra Vulkan driver can hand out
     * an OPAQUE_FD that V4L2 can consume as a DMABUF input buffer. */
    void probeExportFd();
    bool ensureBuffers(unsigned width, unsigned height, bool is16bit);

    /* Record mCmdBuf with pipeline+descriptor bind and a single compute
     * dispatch sized for (width, height); optionally append an image→buffer
     * copy of mScratchImg into mOutBuf so CPU can read the result via mOutMap.
     * Submits to mQueue signalling `fence` (VK_NULL_HANDLE = no fence). */
    void recordAndSubmit(unsigned width, unsigned height, VkFence fence,
                          bool copyToOutBuf);

    VulkanLoader *mLoader;
    VulkanPfn    *mPfn;

    bool mReady;
    unsigned mBufWidth, mBufHeight;

    VkInstance mInstance;
    VkPhysicalDevice mPhysDev;
    VkDevice mDevice;
    VkQueue mQueue;
    uint32_t mQueueFamily;

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

    /* Ring of exportable Bayer input buffers (dma-buf fds handed to V4L2). */
    VkBuffer       mInBuf[kInputBufferCount];
    VkDeviceMemory mInMem[kInputBufferCount];
    void          *mInMap[kInputBufferCount];

    VkBuffer mOutBuf, mParamBuf;
    VkDeviceMemory mOutMem, mParamMem;
    size_t mInSize, mOutSize;
    void *mOutMap, *mParamMap;

    /* Shader output image for CPU-readback paths; copied to mOutBuf after dispatch. */
    VkImage        mScratchImg;
    VkDeviceMemory mScratchMem;
    VkImageView    mScratchView;

    VkFence mFence;
    /* Set by the async processToGralloc path to indicate mFence / mCmdBuf /
     * mInMap are still in use by GPU. Drained at the start of the next call. */
    bool mPrevPending;

    struct IspParams {
        uint32_t width;
        uint32_t height;
        uint32_t bayerPhase;
        uint32_t is16bit;
        uint32_t wbR, wbG, wbB;
        uint32_t doIsp;
        int32_t ccm[9];
        uint32_t gammaLut[64];
    };

    void fillParams(IspParams *p, unsigned w, unsigned h, bool is16, uint32_t pixFmt);
    void updateAwb(const uint8_t *raw, unsigned w, unsigned h, bool is16, uint32_t pixFmt);

    IspParams mParamsTemplate;
    bool mParamsTemplateReady;

    static uint8_t sGammaLut[256];
    static bool sGammaReady;
    static void initGamma();

    bool mNativeBufferAvail;

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
