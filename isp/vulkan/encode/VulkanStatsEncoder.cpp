#define LOG_TAG "Cam-VulkanStatsEnc"
#include <utils/Log.h>
#include <cstring>
#include <cassert>

#include "VulkanStatsEncoder.h"
#include "../runtime/VulkanDeviceState.h"
#include "../runtime/loader/VulkanPfn.h"
#include "../shaders/StatsCompute.h"

namespace android {

struct StatsPushConstants {
    int32_t imgW;
    int32_t imgH;
};

VulkanStatsEncoder::VulkanStatsEncoder(VulkanDeviceState &d)
    : dev(d)
    , shader(VK_NULL_HANDLE)
    , descLayout(VK_NULL_HANDLE)
    , pipeLayout(VK_NULL_HANDLE)
    , pipeline(VK_NULL_HANDLE)
    , descPool(VK_NULL_HANDLE)
    , descSet(VK_NULL_HANDLE)
    , buf(VK_NULL_HANDLE)
    , bufMem(VK_NULL_HANDLE)
    , bufMap(NULL)
    , bufSize(0)
    , scratchView(VK_NULL_HANDLE)
    , scratchSampler(VK_NULL_HANDLE) {}

VulkanStatsEncoder::~VulkanStatsEncoder() { destroy(); }

bool VulkanStatsEncoder::init() {
    if (pipeline != VK_NULL_HANDLE) return true;
    if (!dev.isReady()) return false;

    if (!createShader() ||
        !createDescriptorLayout() ||
        !createPipeline() ||
        !allocateDescriptor()) {
        destroy();
        return false;
    }
    return true;
}

bool VulkanStatsEncoder::createShader() {
    VkShaderModuleCreateInfo smi = {};
    smi.sType    = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    smi.codeSize = strlen(kStatsComputeGlsl);
    smi.pCode    = (const uint32_t *)kStatsComputeGlsl;
    if (dev.pfn()->CreateShaderModule(dev.device(), &smi, NULL, &shader) != VK_SUCCESS) {
        ALOGE("StatsEnc: CreateShaderModule failed");
        return false;
    }
    return true;
}

bool VulkanStatsEncoder::createDescriptorLayout() {
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
    if (dev.pfn()->CreateDescriptorSetLayout(dev.device(), &dslci, NULL, &descLayout) != VK_SUCCESS) {
        ALOGE("StatsEnc: CreateDescriptorSetLayout failed");
        return false;
    }

    VkPushConstantRange pc = {};
    pc.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    pc.offset     = 0;
    pc.size       = sizeof(StatsPushConstants);

    VkPipelineLayoutCreateInfo plci = {};
    plci.sType                  = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    plci.setLayoutCount         = 1;
    plci.pSetLayouts            = &descLayout;
    plci.pushConstantRangeCount = 1;
    plci.pPushConstantRanges    = &pc;
    if (dev.pfn()->CreatePipelineLayout(dev.device(), &plci, NULL, &pipeLayout) != VK_SUCCESS) {
        ALOGE("StatsEnc: CreatePipelineLayout failed");
        return false;
    }
    return true;
}

bool VulkanStatsEncoder::createPipeline() {
    VkComputePipelineCreateInfo cpci = {};
    cpci.sType        = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
    cpci.stage.sType  = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    cpci.stage.stage  = VK_SHADER_STAGE_COMPUTE_BIT;
    cpci.stage.module = shader;
    cpci.stage.pName  = "main";
    cpci.layout       = pipeLayout;
    if (dev.pfn()->CreateComputePipelines(dev.device(), VK_NULL_HANDLE, 1, &cpci, NULL, &pipeline) != VK_SUCCESS) {
        ALOGE("StatsEnc: CreateComputePipelines failed");
        return false;
    }
    return true;
}

bool VulkanStatsEncoder::allocateDescriptor() {
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
    if (dev.pfn()->CreateDescriptorPool(dev.device(), &dpci, NULL, &descPool) != VK_SUCCESS) {
        ALOGE("StatsEnc: CreateDescriptorPool failed");
        return false;
    }

    VkDescriptorSetAllocateInfo dsai = {};
    dsai.sType              = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    dsai.descriptorPool     = descPool;
    dsai.descriptorSetCount = 1;
    dsai.pSetLayouts        = &descLayout;
    if (dev.pfn()->AllocateDescriptorSets(dev.device(), &dsai, &descSet) != VK_SUCCESS) {
        ALOGE("StatsEnc: AllocateDescriptorSets failed");
        return false;
    }
    return true;
}

bool VulkanStatsEncoder::ensureBuffers() {
    if (!pipeline) return false;
    if (buf != VK_NULL_HANDLE) return true;

    size_t want = sizeof(IpaStats);
    if (!dev.createBuffer(&buf, &bufMem, want,
                          VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                          VK_BUFFER_USAGE_TRANSFER_DST_BIT)) {
        ALOGE("StatsEnc: createBuffer(%zu) failed", want);
        return false;
    }
    if (dev.pfn()->MapMemory(dev.device(), bufMem, 0, want, 0, &bufMap) != VK_SUCCESS) {
        ALOGE("StatsEnc: MapMemory failed");
        releaseOutputBuffer();
        return false;
    }

    bufSize = want;

    if (scratchView != VK_NULL_HANDLE && scratchSampler != VK_NULL_HANDLE)
        writeDescriptors();
    return true;
}

void VulkanStatsEncoder::bindScratchInput(VkImageView view, VkSampler sampler) {
    scratchView    = view;
    scratchSampler = sampler;
    if (buf != VK_NULL_HANDLE)
        writeDescriptors();
}

void VulkanStatsEncoder::writeDescriptors() {
    VkDescriptorImageInfo sampledInfo = {};
    sampledInfo.sampler     = scratchSampler;
    sampledInfo.imageView   = scratchView;
    sampledInfo.imageLayout = VK_IMAGE_LAYOUT_GENERAL;

    VkDescriptorBufferInfo outInfo = {};
    outInfo.buffer = buf;
    outInfo.range  = bufSize;

    VkWriteDescriptorSet w[2] = {};
    w[0].sType           = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    w[0].dstSet          = descSet;
    w[0].dstBinding      = 0;
    w[0].descriptorCount = 1;
    w[0].descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    w[0].pImageInfo      = &sampledInfo;
    w[1].sType           = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    w[1].dstSet          = descSet;
    w[1].dstBinding      = 1;
    w[1].descriptorCount = 1;
    w[1].descriptorType  = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    w[1].pBufferInfo     = &outInfo;
    dev.pfn()->UpdateDescriptorSets(dev.device(), 2, w, 0, NULL);
}

void VulkanStatsEncoder::recordDispatch(VkCommandBuffer cb,
                                         unsigned width, unsigned height) {
    assert(buf != VK_NULL_HANDLE);

    /* Histogram accumulates via atomicAdd on global memory; patch
     * means and sharpness are overwritten per-workgroup but the
     * whole-buffer zero also covers any stale bits the CPU might
     * otherwise observe. One fill, one barrier — cheap. */
    dev.pfn()->CmdFillBuffer(cb, buf, 0, bufSize, 0);

    VkBufferMemoryBarrier fillBarrier = {};
    fillBarrier.sType               = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
    fillBarrier.srcAccessMask       = VK_ACCESS_TRANSFER_WRITE_BIT;
    fillBarrier.dstAccessMask       = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT;
    fillBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    fillBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    fillBarrier.buffer              = buf;
    fillBarrier.offset              = 0;
    fillBarrier.size                = bufSize;
    dev.pfn()->CmdPipelineBarrier(cb,
        VK_PIPELINE_STAGE_TRANSFER_BIT,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
        0, 0, NULL, 1, &fillBarrier, 0, NULL);

    dev.pfn()->CmdBindPipeline(cb, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline);
    dev.pfn()->CmdBindDescriptorSets(cb, VK_PIPELINE_BIND_POINT_COMPUTE,
                                      pipeLayout, 0, 1, &descSet, 0, NULL);

    StatsPushConstants pc = {};
    pc.imgW = (int32_t)width;
    pc.imgH = (int32_t)height;
    dev.pfn()->CmdPushConstants(cb, pipeLayout, VK_SHADER_STAGE_COMPUTE_BIT,
                                 0, sizeof(pc), &pc);

    /* One workgroup per output patch — shader uses gl_WorkGroupID to
     * derive patch extents from the image size pushed above. */
    dev.pfn()->CmdDispatch(cb, 16, 16, 1);

    /* Output is host-read on the CPU after the whole submit's fence
     * signals. Barrier protects the CPU read from the GPU write. */
    VkBufferMemoryBarrier hostBarrier = {};
    hostBarrier.sType               = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
    hostBarrier.srcAccessMask       = VK_ACCESS_SHADER_WRITE_BIT;
    hostBarrier.dstAccessMask       = VK_ACCESS_HOST_READ_BIT;
    hostBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    hostBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    hostBarrier.buffer              = buf;
    hostBarrier.offset              = 0;
    hostBarrier.size                = bufSize;
    dev.pfn()->CmdPipelineBarrier(cb,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
        VK_PIPELINE_STAGE_HOST_BIT,
        0, 0, NULL, 1, &hostBarrier, 0, NULL);
}

void VulkanStatsEncoder::invalidateForCpu() {
    if (!bufMem) return;
    VkMappedMemoryRange r = {};
    r.sType  = VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE;
    r.memory = bufMem;
    r.size   = VK_WHOLE_SIZE;
    dev.pfn()->InvalidateMappedMemoryRanges(dev.device(), 1, &r);
}

void VulkanStatsEncoder::releaseOutputBuffer() {
    if (bufMap) {
        dev.pfn()->UnmapMemory(dev.device(), bufMem);
        bufMap = NULL;
    }
    dev.destroyBuffer(buf, bufMem);
    buf     = VK_NULL_HANDLE;
    bufMem  = VK_NULL_HANDLE;
    bufSize = 0;
}

void VulkanStatsEncoder::destroy() {
    if (!dev.isReady() || !dev.pfn()) {
        buf = VK_NULL_HANDLE; bufMem = VK_NULL_HANDLE; bufMap = NULL;
        bufSize = 0;
        return;
    }

    releaseOutputBuffer();

    if (descPool)   { dev.pfn()->DestroyDescriptorPool(dev.device(), descPool, NULL);         descPool = VK_NULL_HANDLE; }
    if (pipeline)   { dev.pfn()->DestroyPipeline(dev.device(), pipeline, NULL);                pipeline = VK_NULL_HANDLE; }
    if (pipeLayout) { dev.pfn()->DestroyPipelineLayout(dev.device(), pipeLayout, NULL);        pipeLayout = VK_NULL_HANDLE; }
    if (descLayout) { dev.pfn()->DestroyDescriptorSetLayout(dev.device(), descLayout, NULL);   descLayout = VK_NULL_HANDLE; }
    if (shader)     { dev.pfn()->DestroyShaderModule(dev.device(), shader, NULL);              shader = VK_NULL_HANDLE; }

    descSet        = VK_NULL_HANDLE;
    scratchView    = VK_NULL_HANDLE;
    scratchSampler = VK_NULL_HANDLE;
}

} /* namespace android */
