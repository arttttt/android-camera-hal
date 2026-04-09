#ifndef GPU_ISP_H
#define GPU_ISP_H

#include <stdint.h>
#include <vulkan/vulkan.h>

namespace android {

class GpuIsp {
public:
    GpuIsp();
    ~GpuIsp();

    bool init();
    void destroy();
    bool isReady() const { return mReady; }

    /*
     * Process Bayer frame → RGBA output on GPU.
     * src: raw Bayer 10-bit (16-bit LE) or 8-bit
     * dst: RGBA8888 output
     * Returns false on error (caller should fallback to CPU).
     */
    bool process(const uint8_t *src, uint8_t *dst,
                 unsigned width, unsigned height,
                 uint32_t pixFmt, bool doIsp,
                 const int16_t *ccm,
                 unsigned wbR, unsigned wbG, unsigned wbB);

private:
    bool createInstance();
    bool createDevice();
    bool createPipeline();
    bool createBuffers(unsigned width, unsigned height, bool is16bit);
    void destroyBuffers();

    bool mReady;
    unsigned mWidth;
    unsigned mHeight;

    VkInstance mInstance;
    VkPhysicalDevice mPhysDevice;
    VkDevice mDevice;
    VkQueue mQueue;
    uint32_t mQueueFamily;

    VkPipeline mPipeline;
    VkPipelineLayout mPipelineLayout;
    VkDescriptorSetLayout mDescSetLayout;
    VkDescriptorPool mDescPool;
    VkDescriptorSet mDescSet;
    VkShaderModule mShaderModule;

    VkCommandPool mCmdPool;
    VkCommandBuffer mCmdBuf;

    /* Input/output buffers */
    VkBuffer mInputBuf;
    VkDeviceMemory mInputMem;
    VkBuffer mOutputBuf;
    VkDeviceMemory mOutputMem;
    VkBuffer mParamsBuf;
    VkDeviceMemory mParamsMem;

    size_t mInputSize;
    size_t mOutputSize;

    uint32_t findMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags props);
};

}; /* namespace android */

#endif // GPU_ISP_H
