#define LOG_TAG "Cam-VulkanYuvEnc"
#include <utils/Log.h>
#include <cstring>
#include <cassert>

#include "VulkanYuvEncoder.h"
#include "../runtime/VulkanDeviceState.h"
#include "../runtime/loader/VulkanPfn.h"
#include "../shaders/RgbaToNv12.h"

namespace android {

struct RgbaToNv12PushConstants {
    int32_t w;
    int32_t h;
    int32_t uvBase;
};

VulkanYuvEncoder::VulkanYuvEncoder(VulkanDeviceState &dev)
    : mDev(dev)
    , mShader(VK_NULL_HANDLE)
    , mDescLayout(VK_NULL_HANDLE)
    , mPipeLayout(VK_NULL_HANDLE)
    , mPipeline(VK_NULL_HANDLE)
    , mDescPool(VK_NULL_HANDLE)
    , mDescSet(VK_NULL_HANDLE)
    , mBuf(VK_NULL_HANDLE)
    , mBufMem(VK_NULL_HANDLE)
    , mBufMap(NULL)
    , mBufSize(0)
    , mWidth(0)
    , mHeight(0)
    , mScratchView(VK_NULL_HANDLE)
    , mScratchSampler(VK_NULL_HANDLE) {}

VulkanYuvEncoder::~VulkanYuvEncoder() { destroy(); }

bool VulkanYuvEncoder::init() {
    if (mPipeline != VK_NULL_HANDLE) return true;
    if (!mDev.isReady()) return false;

    if (!createShader() ||
        !createDescriptorLayout() ||
        !createPipeline() ||
        !allocateDescriptor()) {
        destroy();
        return false;
    }
    return true;
}

bool VulkanYuvEncoder::createShader() {
    VkShaderModuleCreateInfo smi = {};
    smi.sType    = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    smi.codeSize = strlen(kRgbaToNv12ComputeGlsl);
    smi.pCode    = (const uint32_t *)kRgbaToNv12ComputeGlsl;
    if (mDev.pfn()->CreateShaderModule(mDev.device(), &smi, NULL, &mShader) != VK_SUCCESS) {
        ALOGE("RgbaToNv12 shader compile failed");
        return false;
    }
    return true;
}

bool VulkanYuvEncoder::createDescriptorLayout() {
    /* binding 0 — combined image sampler, scratch (compute read).
     * binding 1 — storage buffer, NV12 output (compute write). */
    VkDescriptorSetLayoutBinding b[2] = {};
    b[0].binding         = 0;
    b[0].descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    b[0].descriptorCount = 1;
    b[0].stageFlags      = VK_SHADER_STAGE_COMPUTE_BIT;
    b[1].binding         = 1;
    b[1].descriptorType  = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    b[1].descriptorCount = 1;
    b[1].stageFlags      = VK_SHADER_STAGE_COMPUTE_BIT;

    VkDescriptorSetLayoutCreateInfo dslci = {};
    dslci.sType        = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    dslci.bindingCount = 2;
    dslci.pBindings    = b;
    if (mDev.pfn()->CreateDescriptorSetLayout(mDev.device(), &dslci, NULL, &mDescLayout) != VK_SUCCESS) {
        ALOGE("YuvEnc CreateDescriptorSetLayout failed");
        return false;
    }

    VkPushConstantRange pc = {};
    pc.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    pc.offset     = 0;
    pc.size       = sizeof(RgbaToNv12PushConstants);

    VkPipelineLayoutCreateInfo plci = {};
    plci.sType                  = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    plci.setLayoutCount         = 1;
    plci.pSetLayouts            = &mDescLayout;
    plci.pushConstantRangeCount = 1;
    plci.pPushConstantRanges    = &pc;
    if (mDev.pfn()->CreatePipelineLayout(mDev.device(), &plci, NULL, &mPipeLayout) != VK_SUCCESS) {
        ALOGE("YuvEnc CreatePipelineLayout failed");
        return false;
    }
    return true;
}

bool VulkanYuvEncoder::createPipeline() {
    VkComputePipelineCreateInfo cpci = {};
    cpci.sType        = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
    cpci.stage.sType  = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    cpci.stage.stage  = VK_SHADER_STAGE_COMPUTE_BIT;
    cpci.stage.module = mShader;
    cpci.stage.pName  = "main";
    cpci.layout       = mPipeLayout;
    if (mDev.pfn()->CreateComputePipelines(mDev.device(), VK_NULL_HANDLE, 1, &cpci, NULL, &mPipeline) != VK_SUCCESS) {
        ALOGE("YuvEnc CreateComputePipelines failed");
        return false;
    }
    return true;
}

bool VulkanYuvEncoder::allocateDescriptor() {
    VkDescriptorPoolSize ps[2] = {};
    ps[0].type            = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    ps[0].descriptorCount = 1;
    ps[1].type            = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    ps[1].descriptorCount = 1;

    VkDescriptorPoolCreateInfo dpci = {};
    dpci.sType         = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    dpci.maxSets       = 1;
    dpci.poolSizeCount = 2;
    dpci.pPoolSizes    = ps;
    if (mDev.pfn()->CreateDescriptorPool(mDev.device(), &dpci, NULL, &mDescPool) != VK_SUCCESS) {
        ALOGE("YuvEnc CreateDescriptorPool failed");
        return false;
    }

    VkDescriptorSetAllocateInfo dsai = {};
    dsai.sType              = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    dsai.descriptorPool     = mDescPool;
    dsai.descriptorSetCount = 1;
    dsai.pSetLayouts        = &mDescLayout;
    if (mDev.pfn()->AllocateDescriptorSets(mDev.device(), &dsai, &mDescSet) != VK_SUCCESS) {
        ALOGE("YuvEnc AllocateDescriptorSets failed");
        return false;
    }
    return true;
}

bool VulkanYuvEncoder::ensureBuffers(unsigned width, unsigned height) {
    if (!mPipeline) return false;
    assert(width % 4 == 0 && height % 2 == 0);

    size_t want = (size_t)width * height * 3 / 2;
    if (mWidth == width && mHeight == height && mBufSize >= want) return true;

    releaseOutputBuffer();

    if (!mDev.createBuffer(&mBuf, &mBufMem, want, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT)) {
        ALOGE("YuvEnc createBuffer(%zu) failed", want);
        return false;
    }
    if (mDev.pfn()->MapMemory(mDev.device(), mBufMem, 0, want, 0, &mBufMap) != VK_SUCCESS) {
        ALOGE("YuvEnc MapMemory failed");
        releaseOutputBuffer();
        return false;
    }

    mBufSize = want;
    mWidth   = width;
    mHeight  = height;

    if (mScratchView != VK_NULL_HANDLE && mScratchSampler != VK_NULL_HANDLE)
        writeDescriptors();
    return true;
}

void VulkanYuvEncoder::bindScratchInput(VkImageView scratchView, VkSampler sampler) {
    mScratchView    = scratchView;
    mScratchSampler = sampler;
    if (mBuf != VK_NULL_HANDLE)
        writeDescriptors();
}

void VulkanYuvEncoder::writeDescriptors() {
    VkDescriptorImageInfo sampledInfo = {};
    sampledInfo.sampler     = mScratchSampler;
    sampledInfo.imageView   = mScratchView;
    sampledInfo.imageLayout = VK_IMAGE_LAYOUT_GENERAL;

    VkDescriptorBufferInfo outInfo = {};
    outInfo.buffer = mBuf;
    outInfo.range  = mBufSize;

    VkWriteDescriptorSet w[2] = {};
    w[0].sType           = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    w[0].dstSet          = mDescSet;
    w[0].dstBinding      = 0;
    w[0].descriptorCount = 1;
    w[0].descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    w[0].pImageInfo      = &sampledInfo;
    w[1].sType           = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    w[1].dstSet          = mDescSet;
    w[1].dstBinding      = 1;
    w[1].descriptorCount = 1;
    w[1].descriptorType  = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    w[1].pBufferInfo     = &outInfo;
    mDev.pfn()->UpdateDescriptorSets(mDev.device(), 2, w, 0, NULL);
}

void VulkanYuvEncoder::recordDispatch(VkCommandBuffer cb,
                                      unsigned width, unsigned height) {
    assert(width == mWidth && height == mHeight);

    mDev.pfn()->CmdBindPipeline(cb, VK_PIPELINE_BIND_POINT_COMPUTE, mPipeline);
    mDev.pfn()->CmdBindDescriptorSets(cb, VK_PIPELINE_BIND_POINT_COMPUTE,
                                      mPipeLayout, 0, 1, &mDescSet, 0, NULL);

    RgbaToNv12PushConstants pc = {};
    pc.w      = (int32_t)width;
    pc.h      = (int32_t)height;
    pc.uvBase = (int32_t)(width * height / 4);
    mDev.pfn()->CmdPushConstants(cb, mPipeLayout, VK_SHADER_STAGE_COMPUTE_BIT,
                                 0, sizeof(pc), &pc);

    /* 8×8 workgroup, one invocation per 4×2 Y block. */
    unsigned gx = (width / 4 + 7) / 8;
    unsigned gy = (height / 2 + 7) / 8;
    mDev.pfn()->CmdDispatch(cb, gx, gy, 1);
}

void VulkanYuvEncoder::invalidateForCpu() {
    if (!mBufMem) return;
    VkMappedMemoryRange r = {};
    r.sType  = VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE;
    r.memory = mBufMem;
    r.size   = VK_WHOLE_SIZE;
    mDev.pfn()->InvalidateMappedMemoryRanges(mDev.device(), 1, &r);
}

void VulkanYuvEncoder::releaseOutputBuffer() {
    if (mBufMap) {
        mDev.pfn()->UnmapMemory(mDev.device(), mBufMem);
        mBufMap = NULL;
    }
    mDev.destroyBuffer(mBuf, mBufMem);
    mBuf     = VK_NULL_HANDLE;
    mBufMem  = VK_NULL_HANDLE;
    mBufSize = 0;
    mWidth   = 0;
    mHeight  = 0;
}

void VulkanYuvEncoder::destroy() {
    if (!mDev.isReady() || !mDev.pfn()) {
        mBuf = VK_NULL_HANDLE; mBufMem = VK_NULL_HANDLE; mBufMap = NULL;
        mBufSize = 0; mWidth = 0; mHeight = 0;
        return;
    }

    releaseOutputBuffer();

    if (mDescPool)   { mDev.pfn()->DestroyDescriptorPool(mDev.device(), mDescPool, NULL);         mDescPool = VK_NULL_HANDLE; }
    if (mPipeline)   { mDev.pfn()->DestroyPipeline(mDev.device(), mPipeline, NULL);               mPipeline = VK_NULL_HANDLE; }
    if (mPipeLayout) { mDev.pfn()->DestroyPipelineLayout(mDev.device(), mPipeLayout, NULL);       mPipeLayout = VK_NULL_HANDLE; }
    if (mDescLayout) { mDev.pfn()->DestroyDescriptorSetLayout(mDev.device(), mDescLayout, NULL);  mDescLayout = VK_NULL_HANDLE; }
    if (mShader)     { mDev.pfn()->DestroyShaderModule(mDev.device(), mShader, NULL);             mShader = VK_NULL_HANDLE; }

    mDescSet        = VK_NULL_HANDLE;
    mScratchView    = VK_NULL_HANDLE;
    mScratchSampler = VK_NULL_HANDLE;
}

}; /* namespace android */
