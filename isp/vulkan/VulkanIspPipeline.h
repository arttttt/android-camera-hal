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

private:
    bool createBuffer(VkBuffer *buf, VkDeviceMemory *mem,
                      VkDeviceSize size, VkBufferUsageFlags usage);
    void destroyBuffer(VkBuffer buf, VkDeviceMemory mem);
    uint32_t findMemoryType(uint32_t filter, VkMemoryPropertyFlags props);
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
    VkDescriptorSetLayout mDescLayout;
    VkPipelineLayout mPipeLayout;
    VkPipeline mPipeline;
    VkDescriptorPool mDescPool;
    VkDescriptorSet mDescSet;
    VkCommandPool mCmdPool;
    VkCommandBuffer mCmdBuf;

    VkBuffer mInBuf, mOutBuf, mParamBuf;
    VkDeviceMemory mInMem, mOutMem, mParamMem;
    size_t mInSize, mOutSize;
    void *mInMap, *mOutMap, *mParamMap;

    /* Shader output image for CPU-readback paths; copied to mOutBuf after dispatch. */
    VkImage        mScratchImg;
    VkDeviceMemory mScratchMem;
    VkImageView    mScratchView;

    VkFence mFence;
    uint8_t *mPrevDst;
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

    /* Cached VkImage + view per gralloc buffer_handle_t for zero-copy output.
     * Gralloc owns the backing memory; we only track the Vulkan wrappers. */
    struct GrallocEntry {
        VkImage image;
        VkImageView view;
        bool layoutReady;  /* UNDEFINED → GENERAL transition has been submitted */
    };
    std::unordered_map<const native_handle_t *, GrallocEntry> mGrallocImages;

    bool getOrCreateGrallocImage(ANativeWindowBuffer *anwb,
                                  unsigned width, unsigned height,
                                  GrallocEntry **outEntry);
    void clearGrallocImages();
};

}; /* namespace android */

#endif // VULKAN_ISP_PIPELINE_H
