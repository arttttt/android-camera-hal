#define LOG_TAG "Cam-VulkanISP"
#include <utils/Log.h>
#include <cstring>
#include <errno.h>
#include <unistd.h>
#include <linux/videodev2.h>
#include <math.h>
#include <poll.h>
#include <time.h>

static inline int64_t nowMs() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000LL + ts.tv_nsec / 1000000;
}

#include <system/window.h>
#include "VulkanIspPipeline.h"
#include "runtime/loader/VulkanPfn.h"
#include "shaders/DemosaicCompute.h"
#include "shaders/Blit.h"

namespace android {

VulkanIspPipeline::VulkanIspPipeline()
    : mInputRing(mDeviceState)
    , mReady(false), mBufWidth(0), mBufHeight(0)
    , mShader(VK_NULL_HANDLE), mVertShader(VK_NULL_HANDLE), mFragShader(VK_NULL_HANDLE)
    , mDescLayout(VK_NULL_HANDLE)
    , mPipeLayout(VK_NULL_HANDLE), mPipeline(VK_NULL_HANDLE)
    , mBlitPipeline(VK_NULL_HANDLE), mRenderPass(VK_NULL_HANDLE)
    , mDescPool(VK_NULL_HANDLE)
    , mScratchSampler(VK_NULL_HANDLE)
    , mCmdPool(VK_NULL_HANDLE)
    , mOutBuf(VK_NULL_HANDLE)
    , mOutMem(VK_NULL_HANDLE)
    , mOutSize(0)
    , mOutMap(NULL)
    , mScratchImg(VK_NULL_HANDLE), mScratchMem(VK_NULL_HANDLE), mScratchView(VK_NULL_HANDLE)
    , mNextSlot(0)
    , mGrallocCache(mDeviceState)
    , mYuvEncoder(mDeviceState)
    , mJpegRingW(0)
    , mJpegRingH(0)
    , mTimeQuery(VK_NULL_HANDLE)
{
    for (size_t s = 0; s < SLOT_COUNT; s++) {
        mJpegRing[s].buf    = VK_NULL_HANDLE;
        mJpegRing[s].mem    = VK_NULL_HANDLE;
        mJpegRing[s].mapped = NULL;
        mJpegRing[s].size   = 0;
        mJpegRing[s].inUse  = false;
    }
    for (size_t s = 0; s < SLOT_COUNT; s++) {
        mDescSet[s]     = VK_NULL_HANDLE;
        mCmdBuf[s]      = VK_NULL_HANDLE;
        mParamBuf[s]    = VK_NULL_HANDLE;
        mParamMem[s]    = VK_NULL_HANDLE;
        mParamMap[s]    = NULL;
        mFence[s]       = VK_NULL_HANDLE;
        mSlotSyncFd[s]  = -1;
    }
}

VulkanIspPipeline::~VulkanIspPipeline() { destroy(); }

/* --- helpers (no VK calls) --- */

void VulkanIspPipeline::buildGammaLut() {
    for (int i = 0; i < 256; ++i) {
        float lin = (float)i / 255.0f;
        float enc = (lin <= 0.0031308f)
                    ? lin * 12.92f
                    : 1.055f * powf(lin, 1.0f / 2.4f) - 0.055f;
        int v = (int)(enc * 255.0f + 0.5f);
        if (v < 0) v = 0;
        if (v > 255) v = 255;
        mGammaLut[i] = (uint32_t)v;
    }
}

void VulkanIspPipeline::fillParams(IspParams *p, unsigned w, unsigned h,
                                    bool is16, uint32_t pixFmt) {
    p->reset();
    p->width      = w;
    p->height     = h;
    p->is16bit    = is16 ? 1 : 0;
    p->doIsp      = mEnabled ? 1 : 0;
    p->wbR        = mWbR;
    p->wbG        = mWbG;
    p->wbB        = mWbB;
    p->bayerPhase = IspParams::bayerPhaseFromFourcc(pixFmt);
    p->blackLevel = mBlackLevel;
    const uint32_t maxRaw = is16 ? 1023u : 255u;
    const uint32_t denom  = (mBlackLevel < maxRaw) ? (maxRaw - mBlackLevel) : 1u;
    p->denomRecip255 = 255.0f / (float)denom;
    if (mCcm) {
        for (int i = 0; i < 9; i++)
            p->ccm[i] = mCcm[i];
    }
    memcpy(p->gammaLut, mGammaLut, sizeof(mGammaLut));
}

/* --- frame helpers shared between the process* paths --- */

int VulkanIspPipeline::acquireSlot() {
    int slot = (int)mNextSlot;
    mNextSlot = (mNextSlot + 1) % SLOT_COUNT;
    if (mSlotSyncFd[slot] >= 0) {
        struct pollfd pfd = { mSlotSyncFd[slot], POLLIN, 0 };
        ::poll(&pfd, 1, -1);
        ::close(mSlotSyncFd[slot]);
        mSlotSyncFd[slot] = -1;
    }
    /* Acquire-fence semaphores from the slot's prior submit are consumed
     * at submit-start; once the slot's fence has signalled (poll above or
     * the synchronous WaitForFences in legacy paths) they are safe to
     * destroy. */
    for (VkSemaphore sem : mSlotAcquireSemaphores[slot]) {
        if (sem != VK_NULL_HANDLE)
            mDeviceState.pfn()->DestroySemaphore(mDeviceState.device(), sem, NULL);
    }
    mSlotAcquireSemaphores[slot].clear();
    return slot;
}

void VulkanIspPipeline::uploadParams(int slot, const IspParams &p) {
    memcpy(mParamMap[slot], &p, sizeof(IspParams));

    VkMappedMemoryRange range = {};
    range.sType  = VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE;
    range.memory = mParamMem[slot];
    range.size   = VK_WHOLE_SIZE;
    mDeviceState.pfn()->FlushMappedMemoryRanges(mDeviceState.device(), 1, &range);
}

void VulkanIspPipeline::rebindInputDescriptor(int slot, int inputSlot) {
    VkDescriptorBufferInfo inInfo = {};
    inInfo.buffer = mInputRing.buffer(inputSlot);
    inInfo.range  = mInputRing.slotSize();

    VkWriteDescriptorSet inWrite = {};
    inWrite.sType           = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    inWrite.dstSet          = mDescSet[slot];
    inWrite.dstBinding      = 0;
    inWrite.descriptorCount = 1;
    inWrite.descriptorType  = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    inWrite.pBufferInfo     = &inInfo;
    mDeviceState.pfn()->UpdateDescriptorSets(mDeviceState.device(), 1, &inWrite, 0, NULL);
}

void VulkanIspPipeline::recordAndSubmit(int slot, unsigned width, unsigned height,
                                         bool copyToOutBuf) {
    VkCommandBuffer cb = mCmdBuf[slot];
    VkCommandBufferBeginInfo beginInfo = {};
    beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

    mDeviceState.pfn()->ResetCommandBuffer(cb, 0);
    mDeviceState.pfn()->BeginCommandBuffer(cb, &beginInfo);
    mDeviceState.pfn()->CmdBindPipeline(cb, VK_PIPELINE_BIND_POINT_COMPUTE, mPipeline);
    mDeviceState.pfn()->CmdBindDescriptorSets(cb, VK_PIPELINE_BIND_POINT_COMPUTE,
                                mPipeLayout, 0, 1, &mDescSet[slot], 0, NULL);
    mDeviceState.pfn()->CmdDispatch(cb, (width + 15) / 16, (height + 15) / 16, 1);

    if (copyToOutBuf) {
        VkImageMemoryBarrier imb = {};
        imb.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
        imb.oldLayout = VK_IMAGE_LAYOUT_GENERAL;
        imb.newLayout = VK_IMAGE_LAYOUT_GENERAL;
        imb.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        imb.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
        imb.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        imb.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        imb.image = mScratchImg;
        imb.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        imb.subresourceRange.levelCount = 1;
        imb.subresourceRange.layerCount = 1;

        mDeviceState.pfn()->CmdPipelineBarrier(cb,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT,
            0, 0, NULL, 0, NULL, 1, &imb);

        VkBufferImageCopy region = {};
        region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        region.imageSubresource.layerCount = 1;
        region.imageExtent.width = width;
        region.imageExtent.height = height;
        region.imageExtent.depth = 1;
        mDeviceState.pfn()->CmdCopyImageToBuffer(cb, mScratchImg, VK_IMAGE_LAYOUT_GENERAL,
                                    mOutBuf, 1, &region);
    }

    mDeviceState.pfn()->EndCommandBuffer(cb);

    VkSubmitInfo si = {};
    si.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    si.commandBufferCount = 1;
    si.pCommandBuffers = &cb;
    mDeviceState.pfn()->QueueSubmit(mDeviceState.queue(), 1, &si, mFence[slot]);
}

/* --- produce-once API --- */

void VulkanIspPipeline::recordDemosaicOpen(int slot, unsigned srcW, unsigned srcH) {
    VkCommandBuffer cb = mCmdBuf[slot];
    VkCommandBufferBeginInfo bi = {};
    bi.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    mDeviceState.pfn()->ResetCommandBuffer(cb, 0);
    mDeviceState.pfn()->BeginCommandBuffer(cb, &bi);

    /* Compute: Bayer → mScratchImg. */
    mDeviceState.pfn()->CmdBindPipeline(cb, VK_PIPELINE_BIND_POINT_COMPUTE, mPipeline);
    mDeviceState.pfn()->CmdBindDescriptorSets(cb, VK_PIPELINE_BIND_POINT_COMPUTE,
                                               mPipeLayout, 0, 1, &mDescSet[slot], 0, NULL);
    mDeviceState.pfn()->CmdDispatch(cb, (srcW + 15) / 16, (srcH + 15) / 16, 1);

    /* scratch SHADER_WRITE → SHADER_READ + TRANSFER_READ — covers
     * fragment-stage blits to gralloc, compute-stage YUV encode, and
     * the transfer-stage CopyImageToBuffer used by blitToJpegCpu. */
    VkImageMemoryBarrier scratchB = {};
    scratchB.sType                       = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    scratchB.oldLayout                   = VK_IMAGE_LAYOUT_GENERAL;
    scratchB.newLayout                   = VK_IMAGE_LAYOUT_GENERAL;
    scratchB.srcAccessMask               = VK_ACCESS_SHADER_WRITE_BIT;
    scratchB.dstAccessMask               = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_TRANSFER_READ_BIT;
    scratchB.srcQueueFamilyIndex         = VK_QUEUE_FAMILY_IGNORED;
    scratchB.dstQueueFamilyIndex         = VK_QUEUE_FAMILY_IGNORED;
    scratchB.image                       = mScratchImg;
    scratchB.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    scratchB.subresourceRange.levelCount = 1;
    scratchB.subresourceRange.layerCount = 1;
    mDeviceState.pfn()->CmdPipelineBarrier(cb,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT
            | VK_PIPELINE_STAGE_TRANSFER_BIT,
        0, 0, NULL, 0, NULL, 1, &scratchB);
}

void VulkanIspPipeline::recordRgbaBlitRenderPass(int slot,
                                                   VulkanGrallocCache::Entry *entry,
                                                   unsigned srcW, unsigned srcH,
                                                   unsigned dstW, unsigned dstH,
                                                   const CropRect &crop) {
    VkCommandBuffer cb = mCmdBuf[slot];

    VkRenderPassBeginInfo rpbi = {};
    rpbi.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    rpbi.renderPass  = mRenderPass;
    rpbi.framebuffer = entry->framebuffer;
    rpbi.renderArea.extent.width  = dstW;
    rpbi.renderArea.extent.height = dstH;

    mDeviceState.pfn()->CmdBeginRenderPass(cb, &rpbi, VK_SUBPASS_CONTENTS_INLINE);
    mDeviceState.pfn()->CmdBindPipeline(cb, VK_PIPELINE_BIND_POINT_GRAPHICS, mBlitPipeline);
    mDeviceState.pfn()->CmdBindDescriptorSets(cb, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                               mPipeLayout, 0, 1, &mDescSet[slot], 0, NULL);

    /* Mirrors the push_constant block declared in shaders/Blit.h. */
    struct BlitPushConstants {
        int32_t cropX, cropY, cropW, cropH;
        int32_t srcW,  srcH,  outW,  outH;
        float   uvOffsetX, uvOffsetY;
        float   uvStepX,   uvStepY;
    };
    BlitPushConstants pc = {};
    pc.cropX = crop.x;
    pc.cropY = crop.y;
    pc.cropW = crop.w;
    pc.cropH = crop.h;
    pc.srcW  = (int32_t)srcW;
    pc.srcH  = (int32_t)srcH;
    pc.outW  = (int32_t)dstW;
    pc.outH  = (int32_t)dstH;
    pc.uvOffsetX = (float)crop.x / (float)srcW;
    pc.uvOffsetY = (float)crop.y / (float)srcH;
    pc.uvStepX   = (float)crop.w / ((float)dstW * (float)srcW);
    pc.uvStepY   = (float)crop.h / ((float)dstH * (float)srcH);
    mDeviceState.pfn()->CmdPushConstants(cb, mPipeLayout,
                                          VK_SHADER_STAGE_FRAGMENT_BIT,
                                          0, sizeof(pc), &pc);

    VkViewport vp = {};
    vp.width    = (float)dstW;
    vp.height   = (float)dstH;
    vp.minDepth = 0.0f;
    vp.maxDepth = 1.0f;
    mDeviceState.pfn()->CmdSetViewport(cb, 0, 1, &vp);

    VkRect2D sc = {};
    sc.extent.width  = dstW;
    sc.extent.height = dstH;
    mDeviceState.pfn()->CmdSetScissor(cb, 0, 1, &sc);

    mDeviceState.pfn()->CmdDraw(cb, 3, 1, 0, 0);
    mDeviceState.pfn()->CmdEndRenderPass(cb);
    entry->layoutReady = true;
}

void VulkanIspPipeline::recordYuvEncodeDispatch(int slot, unsigned width, unsigned height) {
    (void)slot;
    mYuvEncoder.recordDispatch(mCmdBuf[slot], width, height);
}

bool VulkanIspPipeline::importAcquireSemaphore(int acquireFence, VkSemaphore *out) {
    *out = VK_NULL_HANDLE;
    if (!mDeviceState.pfn()->ImportSemaphoreFdKHR) {
        ALOGE("ImportSemaphoreFdKHR PFN unavailable; can't import acquire_fence");
        return false;
    }

    VkSemaphoreCreateInfo sci = {};
    sci.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
    VkSemaphore sem = VK_NULL_HANDLE;
    if (mDeviceState.pfn()->CreateSemaphore(mDeviceState.device(), &sci, NULL, &sem) != VK_SUCCESS) {
        ALOGE("vkCreateSemaphore failed");
        return false;
    }

    VkImportSemaphoreFdInfoKHR ii = {};
    ii.sType      = VK_STRUCTURE_TYPE_IMPORT_SEMAPHORE_FD_INFO_KHR;
    ii.semaphore  = sem;
    ii.flags      = VK_SEMAPHORE_IMPORT_TEMPORARY_BIT_KHR;
    ii.handleType = VK_EXTERNAL_SEMAPHORE_HANDLE_TYPE_SYNC_FD_BIT_KHR;
    ii.fd         = acquireFence;
    if (mDeviceState.pfn()->ImportSemaphoreFdKHR(mDeviceState.device(), &ii) != VK_SUCCESS) {
        ALOGE("vkImportSemaphoreFdKHR failed for fd=%d", acquireFence);
        mDeviceState.pfn()->DestroySemaphore(mDeviceState.device(), sem, NULL);
        return false;
    }
    *out = sem;
    return true;
}

bool VulkanIspPipeline::beginFrame(unsigned srcW, unsigned srcH, uint32_t pixFmt,
                                     int srcInputSlot) {
    if (!mReady) return false;
    if (mRec.active) {
        ALOGE("beginFrame called while a frame is already recording");
        return false;
    }
    if (srcInputSlot < 0 || srcInputSlot >= mInputRing.slotCount())
        return false;

    bool is16 = (pixFmt == V4L2_PIX_FMT_SRGGB10 || pixFmt == V4L2_PIX_FMT_SGRBG10 ||
                 pixFmt == V4L2_PIX_FMT_SGBRG10 || pixFmt == V4L2_PIX_FMT_SBGGR10);
    if (!ensureBuffers(srcW, srcH, is16))
        return false;

    int slot = acquireSlot();

    IspParams params;
    fillParams(&params, srcW, srcH, is16, pixFmt);
    uploadParams(slot, params);
    rebindInputDescriptor(slot, srcInputSlot);

    recordDemosaicOpen(slot, srcW, srcH);

    mRec.active = true;
    mRec.slot   = slot;
    mRec.srcW   = srcW;
    mRec.srcH   = srcH;
    return true;
}

bool VulkanIspPipeline::blitToGralloc(void *nativeBuffer,
                                        unsigned dstW, unsigned dstH,
                                        const CropRect &crop,
                                        int acquireFence,
                                        int *releaseFenceOut) {
    *releaseFenceOut = -1;
    if (!mRec.active || !nativeBuffer || !mDeviceState.nativeBufferAvailable())
        return false;

    ANativeWindowBuffer *anwb = (ANativeWindowBuffer *)nativeBuffer;
    VulkanGrallocCache::Entry *entry = NULL;
    if (!mGrallocCache.getOrCreate(anwb, dstW, dstH, &entry))
        return false;

    if (acquireFence >= 0) {
        VkSemaphore sem = VK_NULL_HANDLE;
        if (!importAcquireSemaphore(acquireFence, &sem)) {
            ::close(acquireFence);
            return false;
        }
        mRec.waitSemaphores.push_back(sem);
    }

    recordRgbaBlitRenderPass(mRec.slot, entry, mRec.srcW, mRec.srcH, dstW, dstH, crop);

    PendingBlit b = { entry, releaseFenceOut };
    mRec.blits.push_back(b);
    return true;
}

bool VulkanIspPipeline::blitToYuv(void *nativeBuffer,
                                    unsigned dstW, unsigned dstH,
                                    const CropRect &crop,
                                    int acquireFence,
                                    int *releaseFenceOut) {
    /* nativeBuffer / crop / acquireFence are gralloc-side concerns: the
     * GPU writes NV12 into mYuvEncoder's host-mapped buffer, and the CPU
     * libyuv repack into the output gralloc happens after endFrame's fence
     * has been reaped (BufferProcessor::processYuvOutput). releaseFenceOut
     * therefore stays -1 — the CPU finalize is synchronous on the
     * caller's thread before it returns to the framework. */
    (void)nativeBuffer;
    (void)crop;
    *releaseFenceOut = -1;
    if (acquireFence >= 0) ::close(acquireFence);

    if (!mRec.active) return false;
    if ((dstW & 3) || (dstH & 1)) {
        ALOGE("blitToYuv: %ux%u not a multiple of 4x2", dstW, dstH);
        return false;
    }
    /* For now YUV size must match capture (no scaling in the encoder). */
    if (dstW != mRec.srcW || dstH != mRec.srcH) {
        ALOGE("blitToYuv: dst %ux%u differs from capture %ux%u — cross-resolution YUV not supported",
              dstW, dstH, mRec.srcW, mRec.srcH);
        return false;
    }

    if (!mYuvEncoder.ensureBuffers(dstW, dstH))
        return false;
    mYuvEncoder.bindScratchInput(mScratchView, mScratchSampler);

    recordYuvEncodeDispatch(mRec.slot, dstW, dstH);
    return true;
}

bool VulkanIspPipeline::endFrame(int *submitFenceOut) {
    if (submitFenceOut) *submitFenceOut = -1;
    if (!mRec.active) {
        ALOGE("endFrame called without an active beginFrame");
        return false;
    }
    int slot = mRec.slot;
    VkCommandBuffer cb = mCmdBuf[slot];
    mDeviceState.pfn()->EndCommandBuffer(cb);

    /* Submit waits on each imported acquire-fence semaphore. Stage mask is
     * ALL_COMMANDS so demosaic + every blit serialises behind the framework's
     * gralloc-readiness; demosaic itself only writes scratch (HAL-owned), so
     * the wait could be tightened to COLOR_ATTACHMENT_OUTPUT, but ALL_COMMANDS
     * keeps the rule trivially correct under future shader changes. */
    std::vector<VkPipelineStageFlags> waitStages(mRec.waitSemaphores.size(),
        VK_PIPELINE_STAGE_ALL_COMMANDS_BIT);

    VkSubmitInfo si = {};
    si.sType                = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    si.commandBufferCount   = 1;
    si.pCommandBuffers      = &cb;
    si.waitSemaphoreCount   = (uint32_t)mRec.waitSemaphores.size();
    si.pWaitSemaphores      = mRec.waitSemaphores.empty() ? NULL : mRec.waitSemaphores.data();
    si.pWaitDstStageMask    = waitStages.empty() ? NULL : waitStages.data();
    if (mDeviceState.pfn()->QueueSubmit(mDeviceState.queue(), 1, &si, mFence[slot]) != VK_SUCCESS) {
        ALOGE("endFrame: vkQueueSubmit failed");
        for (VkSemaphore sem : mRec.waitSemaphores) {
            if (sem != VK_NULL_HANDLE)
                mDeviceState.pfn()->DestroySemaphore(mDeviceState.device(), sem, NULL);
        }
        mRec.reset();
        return false;
    }

    /* Per-output release fence: queue serializes after the submit, so each
     * QueueSignalReleaseImageANDROID hands back a sync_fd that signals
     * once all prior commands (including the blit into this image) have
     * drained. Any consumer waiting on the fd before reading the gralloc
     * is correct. */
    for (const PendingBlit &b : mRec.blits) {
        int fd = -1;
        if (mDeviceState.pfn()->QueueSignalReleaseImageANDROID) {
            VkResult qr = mDeviceState.pfn()->QueueSignalReleaseImageANDROID(
                mDeviceState.queue(), 0, NULL, b.entry->image, &fd);
            if (qr != VK_SUCCESS) {
                ALOGW("vkQueueSignalReleaseImageANDROID failed: %d", (int)qr);
                fd = -1;
            }
        }
        *b.releaseFenceOut = fd;
    }

    /* Export the slot's fence as a sync_fd into mSlotSyncFd[slot] for the
     * next acquireSlot to poll on. vkGetFenceFdKHR with SYNC_FD also
     * resets the fence atomically. The same signal is dup'd into
     * submitFenceOut so the caller's poll set (PipelineThread) sees the
     * same completion event without consuming the slot's tracking fd. */
    if (mDeviceState.pfn()->GetFenceFdKHR) {
        VkFenceGetFdInfoKHR gi = {};
        gi.sType      = VK_STRUCTURE_TYPE_FENCE_GET_FD_INFO_KHR;
        gi.fence      = mFence[slot];
        gi.handleType = VK_EXTERNAL_FENCE_HANDLE_TYPE_SYNC_FD_BIT_KHR;
        int fd = -1;
        if (mDeviceState.pfn()->GetFenceFdKHR(mDeviceState.device(), &gi, &fd) == VK_SUCCESS) {
            mSlotSyncFd[slot] = fd;
            if (submitFenceOut) {
                int dupFd = ::dup(fd);
                if (dupFd < 0) {
                    ALOGW("dup of slot sync_fd failed: errno=%d", errno);
                    *submitFenceOut = -1;
                } else {
                    *submitFenceOut = dupFd;
                }
            }
        } else {
            ALOGW("vkGetFenceFdKHR(SYNC_FD) failed; falling back to inline wait");
            mDeviceState.pfn()->WaitForFences(mDeviceState.device(), 1,
                                               &mFence[slot], VK_TRUE, UINT64_MAX);
            mDeviceState.pfn()->ResetFences(mDeviceState.device(), 1, &mFence[slot]);
        }
    } else {
        mDeviceState.pfn()->WaitForFences(mDeviceState.device(), 1,
                                           &mFence[slot], VK_TRUE, UINT64_MAX);
        mDeviceState.pfn()->ResetFences(mDeviceState.device(), 1, &mFence[slot]);
    }

    /* Hand acquire-fence semaphores over to the slot for delayed
     * destruction at the next acquireSlot of this same slot. */
    for (VkSemaphore sem : mRec.waitSemaphores)
        mSlotAcquireSemaphores[slot].push_back(sem);

    mRec.reset();
    return true;
}

/* --- JPEG snapshot ring --- */

bool VulkanIspPipeline::ensureJpegRing(unsigned w, unsigned h) {
    if (w == mJpegRingW && h == mJpegRingH && mJpegRing[0].buf != VK_NULL_HANDLE)
        return true;
    /* Resize: drop existing buffers (none of them should be inUse if the
     * caller is between frames, which it is at beginFrame time). */
    releaseJpegRing();

    const size_t sz = (size_t)w * h * 4;
    for (size_t s = 0; s < SLOT_COUNT; s++) {
        if (!mDeviceState.createBuffer(&mJpegRing[s].buf, &mJpegRing[s].mem,
                                        sz,
                                        VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                        false)) {
            ALOGE("ensureJpegRing: createBuffer failed at slot %zu", s);
            releaseJpegRing();
            return false;
        }
        if (mDeviceState.pfn()->MapMemory(mDeviceState.device(),
                                           mJpegRing[s].mem, 0, sz, 0,
                                           &mJpegRing[s].mapped) != VK_SUCCESS) {
            ALOGE("ensureJpegRing: MapMemory failed at slot %zu", s);
            releaseJpegRing();
            return false;
        }
        mJpegRing[s].size  = sz;
        mJpegRing[s].inUse = false;
    }
    mJpegRingW = w;
    mJpegRingH = h;
    return true;
}

void VulkanIspPipeline::releaseJpegRing() {
    for (size_t s = 0; s < SLOT_COUNT; s++) {
        if (mJpegRing[s].mapped) {
            mDeviceState.pfn()->UnmapMemory(mDeviceState.device(), mJpegRing[s].mem);
            mJpegRing[s].mapped = NULL;
        }
        mDeviceState.destroyBuffer(mJpegRing[s].buf, mJpegRing[s].mem);
        mJpegRing[s].buf   = VK_NULL_HANDLE;
        mJpegRing[s].mem   = VK_NULL_HANDLE;
        mJpegRing[s].size  = 0;
        mJpegRing[s].inUse = false;
    }
    mJpegRingW = 0;
    mJpegRingH = 0;
}

bool VulkanIspPipeline::blitToJpegCpu(JpegSnapshot *out) {
    if (out) {
        out->rgba     = nullptr;
        out->width    = 0;
        out->height   = 0;
        out->size     = 0;
        out->ringSlot = -1;
    }
    if (!mRec.active) {
        ALOGE("blitToJpegCpu called outside of an active beginFrame");
        return false;
    }
    if (!ensureJpegRing(mRec.srcW, mRec.srcH))
        return false;

    int slot = -1;
    for (size_t s = 0; s < SLOT_COUNT; s++) {
        if (!mJpegRing[s].inUse) {
            slot = (int)s;
            break;
        }
    }
    if (slot < 0) {
        ALOGE("blitToJpegCpu: ring exhausted (all %zu slots inUse)", SLOT_COUNT);
        return false;
    }

    VkBufferImageCopy region = {};
    region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    region.imageSubresource.layerCount = 1;
    region.imageExtent.width           = mRec.srcW;
    region.imageExtent.height          = mRec.srcH;
    region.imageExtent.depth           = 1;
    mDeviceState.pfn()->CmdCopyImageToBuffer(mCmdBuf[mRec.slot],
        mScratchImg, VK_IMAGE_LAYOUT_GENERAL,
        mJpegRing[slot].buf, 1, &region);

    mJpegRing[slot].inUse = true;
    if (out) {
        out->rgba     = (const uint8_t *)mJpegRing[slot].mapped;
        out->width    = mRec.srcW;
        out->height   = mRec.srcH;
        out->size     = mJpegRing[slot].size;
        out->ringSlot = slot;
    }
    return true;
}

void VulkanIspPipeline::invalidateJpegSnapshot(const JpegSnapshot &snap) {
    if (snap.ringSlot < 0 || snap.ringSlot >= (int)SLOT_COUNT)
        return;
    VkMappedMemoryRange r = {};
    r.sType  = VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE;
    r.memory = mJpegRing[snap.ringSlot].mem;
    r.size   = VK_WHOLE_SIZE;
    mDeviceState.pfn()->InvalidateMappedMemoryRanges(mDeviceState.device(), 1, &r);
}

void VulkanIspPipeline::releaseJpegSnapshot(const JpegSnapshot &snap) {
    if (snap.ringSlot < 0 || snap.ringSlot >= (int)SLOT_COUNT)
        return;
    mJpegRing[snap.ringSlot].inUse = false;
}

/* --- init / destroy --- */

bool VulkanIspPipeline::init() {
    if (mReady) return true;

    if (!mDeviceState.init()) {
        destroy();
        return false;
    }
    if (!createShaders() ||
        !createDescriptorLayouts() ||
        !createComputePipeline() ||
        !createGraphicsPipeline() ||
        !allocateDescriptorSet() ||
        !createCommandObjects() ||
        !mYuvEncoder.init()) {
        destroy();
        return false;
    }

    buildGammaLut();

    mReady = true;
    ALOGD("Vulkan ISP initialized");
    return true;
}

bool VulkanIspPipeline::compileShader(const char *glsl, VkShaderModule *out) {
    VkShaderModuleCreateInfo smi = {};
    smi.sType    = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    smi.codeSize = strlen(glsl);
    smi.pCode    = (const uint32_t *)glsl;
    return mDeviceState.pfn()->CreateShaderModule(
               mDeviceState.device(), &smi, NULL, out) == VK_SUCCESS;
}

bool VulkanIspPipeline::createShaders() {
    if (!compileShader(kDemosaicComputeGlsl, &mShader)) {
        ALOGE("compute shader compile failed");
        return false;
    }
    if (!compileShader(kBlitVertexGlsl, &mVertShader)) {
        ALOGE("vertex shader compile failed");
        return false;
    }
    if (!compileShader(kBlitFragmentGlsl, &mFragShader)) {
        ALOGE("fragment shader compile failed");
        return false;
    }
    return true;
}

bool VulkanIspPipeline::createDescriptorLayouts() {
    /* Descriptor set layout:
     *   binding 0 — storage buffer, Bayer input (compute only).
     *   binding 1 — storage image, scratch write (compute only).
     *   binding 2 — storage buffer, IspParams (compute only).
     *   binding 3 — combined image sampler, scratch read for the blit
     *               (fragment only). Backed by the same VkImage+view as
     *               binding 1; the sampler gives the fragment path the
     *               texture cache that imageLoad bypassed. */
    VkDescriptorSetLayoutBinding bindings[4] = {};
    bindings[0].binding = 0;
    bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    bindings[0].descriptorCount = 1;
    bindings[0].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    bindings[1] = bindings[0];
    bindings[1].binding = 1;
    bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
    bindings[2] = bindings[0];
    bindings[2].binding = 2;
    bindings[3] = bindings[0];
    bindings[3].binding = 3;
    bindings[3].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    bindings[3].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

    VkDescriptorSetLayoutCreateInfo dslci = {};
    dslci.sType        = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    dslci.bindingCount = 4;
    dslci.pBindings    = bindings;
    if (mDeviceState.pfn()->CreateDescriptorSetLayout(
            mDeviceState.device(), &dslci, NULL, &mDescLayout) != VK_SUCCESS) {
        ALOGE("vkCreateDescriptorSetLayout failed");
        return false;
    }

    /* Push-constant range for the blit shader: crop rect + scratch +
     * output extents + precomputed scale/offset vec2s (see shaders/
     * Blit.h BlitPC block). 48 bytes, fits well under the 128-byte
     * Vulkan 1.0 guarantee. */
    VkPushConstantRange blitPc = {};
    blitPc.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    blitPc.offset     = 0;
    blitPc.size       = 8 * sizeof(int32_t) + 4 * sizeof(float);

    VkPipelineLayoutCreateInfo plci = {};
    plci.sType                  = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    plci.setLayoutCount         = 1;
    plci.pSetLayouts            = &mDescLayout;
    plci.pushConstantRangeCount = 1;
    plci.pPushConstantRanges    = &blitPc;
    if (mDeviceState.pfn()->CreatePipelineLayout(
            mDeviceState.device(), &plci, NULL, &mPipeLayout) != VK_SUCCESS) {
        ALOGE("vkCreatePipelineLayout failed");
        return false;
    }

    /* Sampler for the scratch read path. Linear filter / clamp-to-edge
     * are the defaults we want for eventual bilinear crop+scale; they
     * have no effect on texelFetch (used today for identity blit). */
    VkSamplerCreateInfo sci = {};
    sci.sType        = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
    sci.magFilter    = VK_FILTER_LINEAR;
    sci.minFilter    = VK_FILTER_LINEAR;
    sci.mipmapMode   = VK_SAMPLER_MIPMAP_MODE_NEAREST;
    sci.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sci.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sci.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    sci.unnormalizedCoordinates = VK_FALSE;
    if (mDeviceState.pfn()->CreateSampler(
            mDeviceState.device(), &sci, NULL, &mScratchSampler) != VK_SUCCESS) {
        ALOGE("vkCreateSampler failed");
        return false;
    }
    return true;
}

bool VulkanIspPipeline::createComputePipeline() {
    VkComputePipelineCreateInfo cpci = {};
    cpci.sType        = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
    cpci.stage.sType  = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    cpci.stage.stage  = VK_SHADER_STAGE_COMPUTE_BIT;
    cpci.stage.module = mShader;
    cpci.stage.pName  = "main";
    cpci.layout       = mPipeLayout;

    if (mDeviceState.pfn()->CreateComputePipelines(
            mDeviceState.device(), VK_NULL_HANDLE, 1, &cpci, NULL, &mPipeline) != VK_SUCCESS) {
        ALOGE("vkCreateComputePipelines failed");
        return false;
    }
    return true;
}

bool VulkanIspPipeline::createGraphicsPipeline() {
    /* Render pass — single RGBA8 colour attachment, DONT_CARE → STORE. The
     * ROP path is the only blocklinear-aware write surface on Tegra, so the
     * blit step needs the full graphics pipeline instead of a plain
     * vkCmdCopyImage. */
    VkAttachmentDescription att = {};
    att.format         = VK_FORMAT_R8G8B8A8_UNORM;
    att.samples        = VK_SAMPLE_COUNT_1_BIT;
    att.loadOp         = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    att.storeOp        = VK_ATTACHMENT_STORE_OP_STORE;
    att.stencilLoadOp  = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    att.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    att.initialLayout  = VK_IMAGE_LAYOUT_UNDEFINED;
    att.finalLayout    = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    VkAttachmentReference attRef = {};
    attRef.attachment = 0;
    attRef.layout     = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    VkSubpassDescription subpass = {};
    subpass.pipelineBindPoint    = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass.colorAttachmentCount = 1;
    subpass.pColorAttachments    = &attRef;

    VkRenderPassCreateInfo rpci = {};
    rpci.sType           = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
    rpci.attachmentCount = 1;
    rpci.pAttachments    = &att;
    rpci.subpassCount    = 1;
    rpci.pSubpasses      = &subpass;
    if (mDeviceState.pfn()->CreateRenderPass(
            mDeviceState.device(), &rpci, NULL, &mRenderPass) != VK_SUCCESS) {
        ALOGE("vkCreateRenderPass failed");
        return false;
    }
    mGrallocCache.setRenderPass(mRenderPass);

    VkPipelineShaderStageCreateInfo stages[2] = {};
    stages[0].sType  = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stages[0].stage  = VK_SHADER_STAGE_VERTEX_BIT;
    stages[0].module = mVertShader;
    stages[0].pName  = "main";
    stages[1].sType  = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stages[1].stage  = VK_SHADER_STAGE_FRAGMENT_BIT;
    stages[1].module = mFragShader;
    stages[1].pName  = "main";

    VkPipelineVertexInputStateCreateInfo vis = {};
    vis.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;

    VkPipelineInputAssemblyStateCreateInfo ias = {};
    ias.sType    = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
    ias.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

    VkPipelineViewportStateCreateInfo vps = {};
    vps.sType         = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
    vps.viewportCount = 1;
    vps.scissorCount  = 1;

    VkPipelineRasterizationStateCreateInfo rs = {};
    rs.sType       = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
    rs.polygonMode = VK_POLYGON_MODE_FILL;
    rs.cullMode    = VK_CULL_MODE_NONE;
    rs.frontFace   = VK_FRONT_FACE_COUNTER_CLOCKWISE;
    rs.lineWidth   = 1.0f;

    VkPipelineMultisampleStateCreateInfo ms = {};
    ms.sType                = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
    ms.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

    VkPipelineColorBlendAttachmentState cba = {};
    cba.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
                         VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;

    VkPipelineColorBlendStateCreateInfo cbs = {};
    cbs.sType           = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
    cbs.attachmentCount = 1;
    cbs.pAttachments    = &cba;

    VkDynamicState dyns[2] = { VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR };
    VkPipelineDynamicStateCreateInfo ds = {};
    ds.sType             = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
    ds.dynamicStateCount = 2;
    ds.pDynamicStates    = dyns;

    VkGraphicsPipelineCreateInfo gpci = {};
    gpci.sType               = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
    gpci.stageCount          = 2;
    gpci.pStages             = stages;
    gpci.pVertexInputState   = &vis;
    gpci.pInputAssemblyState = &ias;
    gpci.pViewportState      = &vps;
    gpci.pRasterizationState = &rs;
    gpci.pMultisampleState   = &ms;
    gpci.pColorBlendState    = &cbs;
    gpci.pDynamicState       = &ds;
    gpci.layout              = mPipeLayout;
    gpci.renderPass          = mRenderPass;
    gpci.subpass             = 0;

    if (mDeviceState.pfn()->CreateGraphicsPipelines(
            mDeviceState.device(), VK_NULL_HANDLE, 1, &gpci, NULL, &mBlitPipeline) != VK_SUCCESS) {
        ALOGE("vkCreateGraphicsPipelines failed");
        return false;
    }
    return true;
}

bool VulkanIspPipeline::allocateDescriptorSet() {
    /* Per-slot sets, each binding: 2 storage buffers (input, params) +
     * 1 storage image (scratch write) + 1 combined image sampler
     * (scratch read). Pool is sized for the whole ring. */
    VkDescriptorPoolSize poolSizes[3] = {};
    poolSizes[0].type            = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    poolSizes[0].descriptorCount = 2 * SLOT_COUNT;
    poolSizes[1].type            = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
    poolSizes[1].descriptorCount = 1 * SLOT_COUNT;
    poolSizes[2].type            = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    poolSizes[2].descriptorCount = 1 * SLOT_COUNT;

    VkDescriptorPoolCreateInfo dpci = {};
    dpci.sType         = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    dpci.maxSets       = SLOT_COUNT;
    dpci.poolSizeCount = 3;
    dpci.pPoolSizes    = poolSizes;
    if (mDeviceState.pfn()->CreateDescriptorPool(
            mDeviceState.device(), &dpci, NULL, &mDescPool) != VK_SUCCESS) {
        ALOGE("vkCreateDescriptorPool failed");
        return false;
    }

    VkDescriptorSetLayout layouts[SLOT_COUNT];
    for (size_t s = 0; s < SLOT_COUNT; s++) layouts[s] = mDescLayout;

    VkDescriptorSetAllocateInfo dsai = {};
    dsai.sType              = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    dsai.descriptorPool     = mDescPool;
    dsai.descriptorSetCount = SLOT_COUNT;
    dsai.pSetLayouts        = layouts;
    if (mDeviceState.pfn()->AllocateDescriptorSets(
            mDeviceState.device(), &dsai, mDescSet) != VK_SUCCESS) {
        ALOGE("vkAllocateDescriptorSets failed");
        return false;
    }
    return true;
}

bool VulkanIspPipeline::createCommandObjects() {
    VkCommandPoolCreateInfo cpi = {};
    cpi.sType            = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    cpi.queueFamilyIndex = mDeviceState.queueFamily();
    cpi.flags            = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    mDeviceState.pfn()->CreateCommandPool(mDeviceState.device(), &cpi, NULL, &mCmdPool);

    VkCommandBufferAllocateInfo cbai = {};
    cbai.sType              = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    cbai.commandPool        = mCmdPool;
    cbai.level              = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    cbai.commandBufferCount = SLOT_COUNT;
    mDeviceState.pfn()->AllocateCommandBuffers(mDeviceState.device(), &cbai, mCmdBuf);

    /* Per-slot persistently-mapped parameter buffers — the compute
     * dispatches of two in-flight submits must see independent
     * IspParams content. */
    for (size_t s = 0; s < SLOT_COUNT; s++) {
        if (!mDeviceState.createBuffer(&mParamBuf[s], &mParamMem[s], sizeof(IspParams),
                                        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT)) {
            ALOGE("Failed to create param buffer slot %zu", s);
            return false;
        }
        mDeviceState.pfn()->MapMemory(mDeviceState.device(), mParamMem[s], 0,
                                       sizeof(IspParams), 0, &mParamMap[s]);
    }

    /* Fences created with SYNC_FD_BIT in pNext so PipelineThread can
     * later export each submit's completion as a sync_fd for poll(). */
    VkExportFenceCreateInfoKHR efci = {};
    efci.sType       = VK_STRUCTURE_TYPE_EXPORT_FENCE_CREATE_INFO_KHR;
    efci.handleTypes = VK_EXTERNAL_FENCE_HANDLE_TYPE_SYNC_FD_BIT_KHR;

    VkFenceCreateInfo fci = {};
    fci.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    fci.pNext = &efci;

    for (size_t s = 0; s < SLOT_COUNT; s++) {
        if (mDeviceState.pfn()->CreateFence(
                mDeviceState.device(), &fci, NULL, &mFence[s]) != VK_SUCCESS) {
            ALOGE("vkCreateFence failed for slot %zu", s);
            return false;
        }
    }

    /* GPU timestamp pool — TIMESTAMPS_PER_SLOT queries per slot, so
     * every in-flight submit owns its own offset and results stay
     * independent even if the ring is wrapped before the CPU has
     * read the previous slot. Skipped when the driver reports
     * timestampValidBits == 0 on the compute queue (timestamps
     * would always read as zero). */
    if (mDeviceState.timestampValidBits() > 0) {
        VkQueryPoolCreateInfo qpci = {};
        qpci.sType      = VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO;
        qpci.queryType  = VK_QUERY_TYPE_TIMESTAMP;
        qpci.queryCount = SLOT_COUNT * TIMESTAMPS_PER_SLOT;
        if (mDeviceState.pfn()->CreateQueryPool(mDeviceState.device(), &qpci, NULL,
                                                 &mTimeQuery) != VK_SUCCESS) {
            ALOGW("CreateQueryPool failed — GPU timing disabled");
            mTimeQuery = VK_NULL_HANDLE;
        }
    } else {
        ALOGW("Queue reports timestampValidBits=0 — GPU timing disabled");
    }
    return true;
}

/* --- per-frame buffer management --- */

bool VulkanIspPipeline::ensureBuffers(unsigned width, unsigned height, bool is16bit) {
    size_t inSize  = width * height * (is16bit ? 2 : 1);
    size_t outSize = width * height * 4;

    bool recreate = (mInputRing.slotSize() < inSize || mOutSize < outSize ||
                     mBufWidth != width || mBufHeight != height);
    if (!recreate) return true;

    /* Drain any in-flight slots before tearing down scratch resources
     * they reference (descriptor sets bind mScratchImg for all slots). */
    waitForPreviousFrame();

    /* Cached gralloc wrappers and output-sized Vulkan objects are all
     * sized for the previous resolution — drop them first. */
    mGrallocCache.clear();
    releaseScratchResources();

    if (!mInputRing.ensureSize(inSize)) {
        ALOGE("VulkanInputRing ensureSize(%zu) failed at %ux%u",
              inSize, width, height);
        return false;
    }
    if (!createOutBuffer(outSize))             return false;
    if (!createScratchImage(width, height))    return false;
    writeStaticDescriptors();

    mOutSize   = outSize;
    mBufWidth  = width;
    mBufHeight = height;
    return true;
}

void VulkanIspPipeline::releaseScratchResources() {
    if (mOutMap) {
        mDeviceState.pfn()->UnmapMemory(mDeviceState.device(), mOutMem);
        mOutMap = NULL;
    }
    mDeviceState.destroyBuffer(mOutBuf, mOutMem);
    mOutBuf = VK_NULL_HANDLE;
    mOutMem = VK_NULL_HANDLE;

    if (mScratchView) {
        mDeviceState.pfn()->DestroyImageView(mDeviceState.device(), mScratchView, NULL);
        mScratchView = VK_NULL_HANDLE;
    }
    if (mScratchImg) {
        mDeviceState.pfn()->DestroyImage(mDeviceState.device(), mScratchImg, NULL);
        mScratchImg = VK_NULL_HANDLE;
    }
    if (mScratchMem) {
        mDeviceState.pfn()->FreeMemory(mDeviceState.device(), mScratchMem, NULL);
        mScratchMem = VK_NULL_HANDLE;
    }
    /* JPEG ring is sized to the scratch dimensions; rebuild on resize. */
    releaseJpegRing();
}

bool VulkanIspPipeline::createOutBuffer(size_t size) {
    if (!mDeviceState.createBuffer(&mOutBuf, &mOutMem, size,
            VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT)) {
        ALOGE("Failed to allocate Vulkan output buffer (%zu bytes)", size);
        return false;
    }
    mDeviceState.pfn()->MapMemory(mDeviceState.device(), mOutMem, 0, size, 0, &mOutMap);
    return true;
}

bool VulkanIspPipeline::createScratchImage(unsigned width, unsigned height) {
    VkImageCreateInfo ici = {};
    ici.sType         = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    ici.imageType     = VK_IMAGE_TYPE_2D;
    ici.format        = VK_FORMAT_R8G8B8A8_UNORM;
    ici.extent.width  = width;
    ici.extent.height = height;
    ici.extent.depth  = 1;
    ici.mipLevels     = 1;
    ici.arrayLayers   = 1;
    ici.samples       = VK_SAMPLE_COUNT_1_BIT;
    ici.tiling        = VK_IMAGE_TILING_OPTIMAL;
    ici.usage         = VK_IMAGE_USAGE_STORAGE_BIT |
                        VK_IMAGE_USAGE_SAMPLED_BIT |
                        VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
    ici.sharingMode   = VK_SHARING_MODE_EXCLUSIVE;
    ici.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    if (mDeviceState.pfn()->CreateImage(mDeviceState.device(), &ici, NULL, &mScratchImg) != VK_SUCCESS) {
        ALOGE("Scratch vkCreateImage failed %ux%u", width, height);
        return false;
    }

    VkMemoryRequirements req;
    mDeviceState.pfn()->GetImageMemoryRequirements(mDeviceState.device(), mScratchImg, &req);

    VkMemoryAllocateInfo ai = {};
    ai.sType           = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    ai.allocationSize  = req.size;
    ai.memoryTypeIndex = mDeviceState.findMemoryType(req.memoryTypeBits,
                                                     VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    if (ai.memoryTypeIndex == UINT32_MAX)
        ai.memoryTypeIndex = mDeviceState.findMemoryType(req.memoryTypeBits, 0);
    if (ai.memoryTypeIndex == UINT32_MAX ||
        mDeviceState.pfn()->AllocateMemory(mDeviceState.device(), &ai, NULL, &mScratchMem) != VK_SUCCESS) {
        ALOGE("Scratch vkAllocateMemory failed");
        return false;
    }
    mDeviceState.pfn()->BindImageMemory(mDeviceState.device(), mScratchImg, mScratchMem, 0);

    VkImageViewCreateInfo vci = {};
    vci.sType                        = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    vci.image                        = mScratchImg;
    vci.viewType                     = VK_IMAGE_VIEW_TYPE_2D;
    vci.format                       = VK_FORMAT_R8G8B8A8_UNORM;
    vci.subresourceRange.aspectMask  = VK_IMAGE_ASPECT_COLOR_BIT;
    vci.subresourceRange.levelCount  = 1;
    vci.subresourceRange.layerCount  = 1;
    if (mDeviceState.pfn()->CreateImageView(mDeviceState.device(), &vci, NULL, &mScratchView) != VK_SUCCESS) {
        ALOGE("Scratch vkCreateImageView failed");
        return false;
    }

    /* One-time UNDEFINED → GENERAL transition so the compute shader can
     * imageStore into the scratch image on the first dispatch. This
     * runs on slot 0 in isolation (no other slot is active during
     * resource allocation), synchronously — it's an init path, not
     * hot. */
    VkCommandBuffer cb = mCmdBuf[0];
    VkCommandBufferBeginInfo bi = {};
    bi.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    mDeviceState.pfn()->ResetCommandBuffer(cb, 0);
    mDeviceState.pfn()->BeginCommandBuffer(cb, &bi);

    VkImageMemoryBarrier imb = {};
    imb.sType                           = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    imb.oldLayout                       = VK_IMAGE_LAYOUT_UNDEFINED;
    imb.newLayout                       = VK_IMAGE_LAYOUT_GENERAL;
    imb.srcAccessMask                   = 0;
    imb.dstAccessMask                   = VK_ACCESS_SHADER_WRITE_BIT;
    imb.srcQueueFamilyIndex             = VK_QUEUE_FAMILY_IGNORED;
    imb.dstQueueFamilyIndex             = VK_QUEUE_FAMILY_IGNORED;
    imb.image                           = mScratchImg;
    imb.subresourceRange.aspectMask     = VK_IMAGE_ASPECT_COLOR_BIT;
    imb.subresourceRange.levelCount     = 1;
    imb.subresourceRange.layerCount     = 1;
    mDeviceState.pfn()->CmdPipelineBarrier(cb,
        VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
        0, 0, NULL, 0, NULL, 1, &imb);
    mDeviceState.pfn()->EndCommandBuffer(cb);

    VkSubmitInfo si = {};
    si.sType              = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    si.commandBufferCount = 1;
    si.pCommandBuffers    = &cb;
    mDeviceState.pfn()->QueueSubmit(mDeviceState.queue(), 1, &si, mFence[0]);
    mDeviceState.pfn()->WaitForFences(mDeviceState.device(), 1, &mFence[0], VK_TRUE, UINT64_MAX);
    mDeviceState.pfn()->ResetFences(mDeviceState.device(), 1, &mFence[0]);
    return true;
}

void VulkanIspPipeline::writeStaticDescriptors() {
    /* For every slot: bind input ring slot 0 (rebound per-frame via
     * rebindInputDescriptor), scratch image for compute write
     * (binding 1) and for fragment sampled read (binding 3) — same
     * VkImage+view, different descriptor type. Param buffer (binding 2)
     * is per-slot (see mParamBuf[]). */
    VkDescriptorImageInfo storageInfo = {};
    storageInfo.imageView   = mScratchView;
    storageInfo.imageLayout = VK_IMAGE_LAYOUT_GENERAL;

    VkDescriptorImageInfo sampledInfo = {};
    sampledInfo.sampler     = mScratchSampler;
    sampledInfo.imageView   = mScratchView;
    sampledInfo.imageLayout = VK_IMAGE_LAYOUT_GENERAL;

    for (size_t s = 0; s < SLOT_COUNT; s++) {
        VkDescriptorBufferInfo inInfo = {};
        inInfo.buffer = mInputRing.buffer(0);
        inInfo.range  = mInputRing.slotSize();

        VkDescriptorBufferInfo paramInfo = {};
        paramInfo.buffer = mParamBuf[s];
        paramInfo.range  = sizeof(IspParams);

        VkWriteDescriptorSet writes[4] = {};
        writes[0].sType           = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[0].dstSet          = mDescSet[s];
        writes[0].dstBinding      = 0;
        writes[0].descriptorCount = 1;
        writes[0].descriptorType  = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[0].pBufferInfo     = &inInfo;

        writes[1].sType           = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[1].dstSet          = mDescSet[s];
        writes[1].dstBinding      = 1;
        writes[1].descriptorCount = 1;
        writes[1].descriptorType  = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
        writes[1].pImageInfo      = &storageInfo;

        writes[2].sType           = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[2].dstSet          = mDescSet[s];
        writes[2].dstBinding      = 2;
        writes[2].descriptorCount = 1;
        writes[2].descriptorType  = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[2].pBufferInfo     = &paramInfo;

        writes[3].sType           = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[3].dstSet          = mDescSet[s];
        writes[3].dstBinding      = 3;
        writes[3].descriptorCount = 1;
        writes[3].descriptorType  = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        writes[3].pImageInfo      = &sampledInfo;

        mDeviceState.pfn()->UpdateDescriptorSets(mDeviceState.device(), 4, writes, 0, NULL);
    }
}

/* --- main processing paths --- */

void VulkanIspPipeline::prewarm(unsigned width, unsigned height, uint32_t pixFmt) {
    if (!mReady) return;

    bool is16 = (pixFmt == V4L2_PIX_FMT_SRGGB10 || pixFmt == V4L2_PIX_FMT_SGRBG10 ||
                 pixFmt == V4L2_PIX_FMT_SGBRG10 || pixFmt == V4L2_PIX_FMT_SBGGR10);
    if (!ensureBuffers(width, height, is16))
        return;

    int slot = acquireSlot();

    IspParams params;
    params.reset();
    params.width   = width;
    params.height  = height;
    params.is16bit = is16 ? 1 : 0;
    params.wbR     = 256;
    params.wbG     = 256;
    params.wbB     = 256;
    uploadParams(slot, params);

    int64_t start = nowMs();
    recordAndSubmit(slot, width, height, false);
    mDeviceState.pfn()->WaitForFences(mDeviceState.device(), 1,
                                       &mFence[slot], VK_TRUE, UINT64_MAX);
    mDeviceState.pfn()->ResetFences(mDeviceState.device(), 1, &mFence[slot]);
    ALOGD("Vulkan ISP prewarm %ux%u is16=%d: %lldms",
          width, height, is16 ? 1 : 0, nowMs() - start);
}

/* --- cleanup --- */

void VulkanIspPipeline::destroy() {
    if (mDeviceState.isReady() && mDeviceState.pfn()->DeviceWaitIdle) {
        mDeviceState.pfn()->DeviceWaitIdle(mDeviceState.device());

        if (mOutMap) {
            mDeviceState.pfn()->UnmapMemory(mDeviceState.device(), mOutMem);
            mOutMap = NULL;
        }
        for (size_t s = 0; s < SLOT_COUNT; s++) {
            if (mParamMap[s]) {
                mDeviceState.pfn()->UnmapMemory(mDeviceState.device(), mParamMem[s]);
                mParamMap[s] = NULL;
            }
        }

        mGrallocCache.clear();
        mYuvEncoder.destroy();
        mInputRing.destroy();
        releaseJpegRing();

        if (mTimeQuery) {
            mDeviceState.pfn()->DestroyQueryPool(mDeviceState.device(), mTimeQuery, NULL);
            mTimeQuery = VK_NULL_HANDLE;
        }

        if (mScratchView) {
            mDeviceState.pfn()->DestroyImageView(mDeviceState.device(), mScratchView, NULL);
            mScratchView = VK_NULL_HANDLE;
        }
        if (mScratchImg) {
            mDeviceState.pfn()->DestroyImage(mDeviceState.device(), mScratchImg, NULL);
            mScratchImg = VK_NULL_HANDLE;
        }
        if (mScratchMem) {
            mDeviceState.pfn()->FreeMemory(mDeviceState.device(), mScratchMem, NULL);
            mScratchMem = VK_NULL_HANDLE;
        }

        mDeviceState.destroyBuffer(mOutBuf, mOutMem);
        mOutBuf = VK_NULL_HANDLE;
        mOutMem = VK_NULL_HANDLE;
        for (size_t s = 0; s < SLOT_COUNT; s++) {
            mDeviceState.destroyBuffer(mParamBuf[s], mParamMem[s]);
            mParamBuf[s] = VK_NULL_HANDLE;
            mParamMem[s] = VK_NULL_HANDLE;
        }

        for (size_t s = 0; s < SLOT_COUNT; s++) {
            if (mFence[s]) {
                mDeviceState.pfn()->DestroyFence(mDeviceState.device(), mFence[s], NULL);
                mFence[s] = VK_NULL_HANDLE;
            }
            for (VkSemaphore sem : mSlotAcquireSemaphores[s]) {
                if (sem != VK_NULL_HANDLE)
                    mDeviceState.pfn()->DestroySemaphore(mDeviceState.device(), sem, NULL);
            }
            mSlotAcquireSemaphores[s].clear();
            if (mSlotSyncFd[s] >= 0) {
                ::close(mSlotSyncFd[s]);
                mSlotSyncFd[s] = -1;
            }
        }
        /* Drop any in-progress recording state (begin without matching end). */
        for (VkSemaphore sem : mRec.waitSemaphores) {
            if (sem != VK_NULL_HANDLE)
                mDeviceState.pfn()->DestroySemaphore(mDeviceState.device(), sem, NULL);
        }
        mRec.reset();
        if (mCmdPool) {
            mDeviceState.pfn()->DestroyCommandPool(mDeviceState.device(), mCmdPool, NULL);
            mCmdPool = VK_NULL_HANDLE;
        }
        if (mDescPool) {
            mDeviceState.pfn()->DestroyDescriptorPool(mDeviceState.device(), mDescPool, NULL);
            mDescPool = VK_NULL_HANDLE;
        }
        if (mBlitPipeline) {
            mDeviceState.pfn()->DestroyPipeline(mDeviceState.device(), mBlitPipeline, NULL);
            mBlitPipeline = VK_NULL_HANDLE;
        }
        if (mPipeline) {
            mDeviceState.pfn()->DestroyPipeline(mDeviceState.device(), mPipeline, NULL);
            mPipeline = VK_NULL_HANDLE;
        }
        if (mRenderPass) {
            mDeviceState.pfn()->DestroyRenderPass(mDeviceState.device(), mRenderPass, NULL);
            mRenderPass = VK_NULL_HANDLE;
        }
        if (mPipeLayout) {
            mDeviceState.pfn()->DestroyPipelineLayout(mDeviceState.device(), mPipeLayout, NULL);
            mPipeLayout = VK_NULL_HANDLE;
        }
        if (mDescLayout) {
            mDeviceState.pfn()->DestroyDescriptorSetLayout(mDeviceState.device(), mDescLayout, NULL);
            mDescLayout = VK_NULL_HANDLE;
        }
        if (mScratchSampler) {
            mDeviceState.pfn()->DestroySampler(mDeviceState.device(), mScratchSampler, NULL);
            mScratchSampler = VK_NULL_HANDLE;
        }
        if (mFragShader) {
            mDeviceState.pfn()->DestroyShaderModule(mDeviceState.device(), mFragShader, NULL);
            mFragShader = VK_NULL_HANDLE;
        }
        if (mVertShader) {
            mDeviceState.pfn()->DestroyShaderModule(mDeviceState.device(), mVertShader, NULL);
            mVertShader = VK_NULL_HANDLE;
        }
        if (mShader) {
            mDeviceState.pfn()->DestroyShaderModule(mDeviceState.device(), mShader, NULL);
            mShader = VK_NULL_HANDLE;
        }
    }

    mDeviceState.destroy();

    mReady = false;
    mOutSize = 0;
    mBufWidth = 0;
    mBufHeight = 0;
    for (size_t s = 0; s < SLOT_COUNT; s++) {
        if (mSlotSyncFd[s] >= 0) {
            ::close(mSlotSyncFd[s]);
            mSlotSyncFd[s] = -1;
        }
    }
    mNextSlot = 0;
}

void VulkanIspPipeline::onSessionClose() {
    /* Release cached VkImage bindings into framework gralloc buffers —
     * the framework invalidates them at session close and will allocate
     * fresh buffers for the next session, so the cache entries would
     * otherwise point at dead handles. Core resources (device, scratch
     * image, input ring, shaders, descriptor pool) survive. */
    if (mDeviceState.isReady() && mDeviceState.pfn()->DeviceWaitIdle) {
        mDeviceState.pfn()->DeviceWaitIdle(mDeviceState.device());
    }
    mGrallocCache.clear();

    /* Drop any per-slot sync_fds from the session that just closed —
     * DeviceWaitIdle above guarantees the corresponding submits are
     * complete, so the fds carry nothing useful, and acquireSlot on
     * the next session would otherwise poll on fds from the previous
     * session before reusing the slot. mNextSlot resets so the ring
     * starts from slot 0 like a fresh init. */
    for (size_t s = 0; s < SLOT_COUNT; s++) {
        if (mSlotSyncFd[s] >= 0) {
            ::close(mSlotSyncFd[s]);
            mSlotSyncFd[s] = -1;
        }
    }
    mNextSlot = 0;
}

void VulkanIspPipeline::waitForPreviousFrame() {
    if (!mReady) return;
    for (size_t s = 0; s < SLOT_COUNT; s++) {
        if (mSlotSyncFd[s] < 0) continue;
        struct pollfd pfd = { mSlotSyncFd[s], POLLIN, 0 };
        ::poll(&pfd, 1, -1);
        ::close(mSlotSyncFd[s]);
        mSlotSyncFd[s] = -1;
    }
}

}; /* namespace android */
