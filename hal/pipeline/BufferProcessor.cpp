#define LOG_TAG "Cam-BufProc"

#include "BufferProcessor.h"

#include <string.h>
#include <linux/videodev2.h>

#include <hardware/gralloc.h>
#include <system/graphics.h>
#include <ui/Rect.h>
#include <ui/GraphicBuffer.h>
#include <ui/GraphicBufferMapper.h>
#include <ui/Fence.h>
#include <utils/Log.h>
#include <utils/Timers.h>
#include <libyuv.h>

#include "IspPipeline.h"
#include "jpeg/JpegEncoder.h"
#include "PipelineContext.h"
#include "DbgUtils.h"

namespace android {

namespace {

/* Timeout for acquire_fence wait — preserves the pre-existing magic
 * number; revisit when fence semantics are formalised. */
constexpr int kAcquireFenceTimeoutMs = 1000;

} /* namespace */

BufferProcessor::BufferProcessor(const Deps &deps)
    : mDeps(deps) {
}

status_t BufferProcessor::waitAcquireFence(const camera3_stream_buffer &srcBuf,
                                            uint32_t frameNumber) {
    /* Acquire fence signals when the gralloc buffer is safe for the
     * HAL to write. On a backed-up compositor this is the main way the
     * framework back-pressures us — measuring it surfaces the case
     * where "GPU isn't the bottleneck, the display is". Only log when
     * the wait is meaningful (>5 ms) to keep the steady-state log
     * readable. */
    int64_t t0 = systemTime();
    sp<Fence> acquireFence = new Fence(srcBuf.acquire_fence);
    status_t e = acquireFence->wait(kAcquireFenceTimeoutMs);
    int64_t waitMs = (systemTime() - t0) / 1000000;
    if (waitMs > 5) {
        ALOGD("acquire_fence wait %lldms  frame %u  buffer %p",
              (long long)waitMs, frameNumber, srcBuf.buffer);
    }
    if (e == TIMED_OUT) {
        ALOGE("buffer %p  frame %-4u  Wait on acquire fence timed out",
              srcBuf.buffer, frameNumber);
    }
    if (e != NO_ERROR)
        return NO_INIT;
    return NO_ERROR;
}

bool BufferProcessor::tryZeroCopy(const camera3_stream_buffer &srcBuf,
                                   const FrameContext &ctx,
                                   OutputState *state,
                                   int *releaseFenceOut) {
    /* The caller guarantees this stream is RGBA_8888 (Bayer sensor =
     * the only source). The shader handles crop + rescale per push-constant.
     * acquire_fence (sync_fd) gets imported as a binary VkSemaphore so the
     * eventual submit GPU-waits on framework readiness without blocking
     * this thread. */
    unsigned streamW = srcBuf.stream->width;
    unsigned streamH = srcBuf.stream->height;

    sp<GraphicBuffer> gb = new GraphicBuffer(streamW, streamH,
        HAL_PIXEL_FORMAT_RGBA_8888,
        GRALLOC_USAGE_HW_TEXTURE | GRALLOC_USAGE_HW_COMPOSER,
        streamW, const_cast<native_handle_t *>(*srcBuf.buffer),
        false);
    bool zcOk = false;
    BENCHMARK_SECTION("Raw->RGBA") {
        CropRect crop = { ctx.cropX, ctx.cropY, ctx.cropW, ctx.cropH };
        zcOk = mDeps.isp->blitToGralloc(gb->getNativeBuffer(),
                                         streamW, streamH, crop,
                                         srcBuf.acquire_fence,
                                         releaseFenceOut);
    }
    if (!zcOk)
        return false;

    state->needsFinalUnlock = false;
    return true;
}

status_t BufferProcessor::lockSwWrite(const camera3_stream_buffer &srcBuf,
                                       uint32_t frameNumber,
                                       uint8_t **outBuf) {
    unsigned streamW = srcBuf.stream->width;
    unsigned streamH = srcBuf.stream->height;
    const Rect rect((int)streamW, (int)streamH);
    status_t e = GraphicBufferMapper::get().lock(*srcBuf.buffer,
        GRALLOC_USAGE_SW_WRITE_OFTEN, rect, (void **)outBuf);
    if (e != NO_ERROR) {
        ALOGE("buffer %p  frame %-4u  lock failed", srcBuf.buffer, frameNumber);
        return NO_INIT;
    }
    return NO_ERROR;
}

status_t BufferProcessor::recordYuvOutput(const camera3_stream_buffer &srcBuf,
                                            const FrameContext &ctx,
                                            uint32_t frameNumber) {
    /* CPU-wait on the framework acquire_fence here so finalizeYuvOutput's
     * lockYCbCr (post-fence-reap) doesn't have to block. The wait is a
     * few ms in steady state; the GPU encode dispatch we record next
     * runs in parallel with subsequent frames. */
    status_t e = waitAcquireFence(srcBuf, frameNumber);
    if (e != NO_ERROR)
        return e;

    unsigned w = srcBuf.stream->width;
    unsigned h = srcBuf.stream->height;
    int releaseFd = -1;
    CropRect crop = { ctx.cropX, ctx.cropY, ctx.cropW, ctx.cropH };
    bool ok = false;
    BENCHMARK_SECTION("Raw->NV12") {
        ok = mDeps.isp->blitToYuv(srcBuf.buffer, w, h, crop, -1, &releaseFd);
    }
    if (!ok) {
        ALOGE("buffer %p  frame %-4u  blitToYuv failed",
              srcBuf.buffer, frameNumber);
        return NO_INIT;
    }
    return NO_ERROR;
}

void BufferProcessor::finalizeYuvOutput(const CaptureRequest::Buffer &outBuf,
                                          uint32_t frameNumber) {
    unsigned w = outBuf.stream->width;
    unsigned h = outBuf.stream->height;

    mDeps.isp->invalidateYuvForCpu();
    const uint8_t *nv12 = mDeps.isp->yuvHostBuffer();
    if (!nv12) {
        ALOGE("buffer %p  frame %-4u  yuvHostBuffer null at finalize",
              outBuf.buffer, frameNumber);
        return;
    }
    const uint8_t *srcY  = nv12;
    const uint8_t *srcUv = nv12 + (size_t)w * h;

    android_ycbcr dst = {};
    const Rect rect((int)w, (int)h);
    status_t e = GraphicBufferMapper::get().lockYCbCr(*outBuf.buffer,
        GRALLOC_USAGE_SW_WRITE_OFTEN, rect, &dst);
    if (e != NO_ERROR) {
        ALOGE("buffer %p  frame %-4u  lockYCbCr failed", outBuf.buffer, frameNumber);
        return;
    }

    uint8_t *dstY  = (uint8_t *)dst.y;
    uint8_t *dstCb = (uint8_t *)dst.cb;
    uint8_t *dstCr = (uint8_t *)dst.cr;

    BENCHMARK_SECTION("NV12->gralloc") {
        if (dst.chroma_step == 2 && dstCb + 1 == dstCr) {
            libyuv::CopyPlane(srcY,  (int)w, dstY,  (int)dst.ystride,
                              (int)w, (int)h);
            libyuv::CopyPlane(srcUv, (int)w, dstCb, (int)dst.cstride,
                              (int)w, (int)h / 2);
        } else if (dst.chroma_step == 1) {
            libyuv::NV12ToI420(srcY, (int)w, srcUv, (int)w,
                               dstY,  (int)dst.ystride,
                               dstCb, (int)dst.cstride,
                               dstCr, (int)dst.cstride,
                               (int)w, (int)h);
        } else {
            ALOGE("buffer %p  frame %-4u  YUV layout not supported "
                  "(chroma_step=%d, cb=%p, cr=%p)",
                  outBuf.buffer, frameNumber,
                  dst.chroma_step, dstCb, dstCr);
        }
    }

    GraphicBufferMapper::get().unlock(*outBuf.buffer);
}

void BufferProcessor::finalizeCpuOutputs(PipelineContext &ctx) {
    if (ctx.errorCode) return;
    for (size_t i = 0; i < ctx.request.outputBuffers.size(); ++i) {
        const CaptureRequest::Buffer &ob = ctx.request.outputBuffers[i];
        if (ob.stream->format != HAL_PIXEL_FORMAT_YCbCr_420_888) continue;
        if (ctx.outputStatuses[i] != CAMERA3_BUFFER_STATUS_OK) continue;
        finalizeYuvOutput(ob, ctx.request.frameNumber);
    }
}

void BufferProcessor::processBlobOutput(uint8_t *buf, const FrameContext &ctx,
                                         const CameraMetadata &cm) {
    BENCHMARK_SECTION("YUV->JPEG") {
        JpegSource jsrc;
        jsrc.frameBuf     = ctx.frameBuf;
        jsrc.srcInputSlot = ctx.frameSlotIdx;
        jsrc.pixFmt       = ctx.pixFmt;
        jsrc.width        = ctx.resW;
        jsrc.height       = ctx.resH;
        mDeps.jpeg->encode(buf, ctx.jpegBufferSize, jsrc, cm);
    }
}

status_t BufferProcessor::processOne(const camera3_stream_buffer &srcBuf,
                                      const FrameContext &ctx,
                                      const CameraMetadata &cm,
                                      uint32_t frameNumber,
                                      OutputState *state,
                                      int *releaseFenceOut) {
    state->needsFinalUnlock = true;
    if (releaseFenceOut) *releaseFenceOut = -1;

    switch (srcBuf.stream->format) {
        case HAL_PIXEL_FORMAT_RGBA_8888:
            /* RGBA zero-copy: GPU writes directly into the gralloc via the
             * blit render pass, framework acquire_fence is imported as a
             * VkSemaphore on the submit, release_fence is exported per
             * output by endFrame. needsFinalUnlock=false — the gralloc is
             * never CPU-locked. */
            if (!tryZeroCopy(srcBuf, ctx, state, releaseFenceOut)) {
                ALOGE("buffer %p  frame %-4u  zero-copy failed",
                      srcBuf.buffer, frameNumber);
                return NO_INIT;
            }
            return NO_ERROR;
        case HAL_PIXEL_FORMAT_YCbCr_420_888: {
            /* GPU encode dispatch is recorded now; libyuv repack into the
             * gralloc happens in finalizeCpuOutputs after the frame's
             * fence has been reaped. needsFinalUnlock=false — finalize
             * does its own lock/unlock and releaseFenceOut stays -1
             * (CPU finalize is synchronous). */
            status_t e = recordYuvOutput(srcBuf, ctx, frameNumber);
            if (e != NO_ERROR) return e;
            state->needsFinalUnlock = false;
            return NO_ERROR;
        }
        case HAL_PIXEL_FORMAT_BLOB: {
            /* JPEG legacy path until the JpegWorker split lands. CPU-wait
             * acquire_fence then run libjpeg synchronously inside
             * processBlobOutput. Demosaic happens via the legacy
             * processToCpu — separate submit from the produce-once one,
             * but harmless: both overwrite the same scratch with the same
             * Bayer + params. */
            status_t e = waitAcquireFence(srcBuf, frameNumber);
            if (e != NO_ERROR) return e;
            uint8_t *buf = NULL;
            e = lockSwWrite(srcBuf, frameNumber, &buf);
            if (e != NO_ERROR) return e;
            processBlobOutput(buf, ctx, cm);
            return NO_ERROR;
        }
        default:
            ALOGE("Unknown pixel format %d in buffer %p (stream %p), ignoring",
                  srcBuf.stream->format, srcBuf.buffer, srcBuf.stream);
            return NO_ERROR;
    }
}

}; /* namespace android */
