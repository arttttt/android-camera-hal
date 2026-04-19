#define LOG_TAG "Cam-BufProc"

#include "BufferProcessor.h"

#include <string.h>
#include <linux/videodev2.h>

#include <hardware/gralloc.h>
#include <ui/Rect.h>
#include <ui/GraphicBuffer.h>
#include <ui/GraphicBufferMapper.h>
#include <ui/Fence.h>
#include <utils/Log.h>

#include "IspPipeline.h"
#include "jpeg/JpegEncoder.h"
#include "3a/AutoFocusController.h"
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
    sp<Fence> acquireFence = new Fence(srcBuf.acquire_fence);
    status_t e = acquireFence->wait(kAcquireFenceTimeoutMs);
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
                                   uint8_t **sharedRgba) {
    /* The caller guarantees this stream is RGBA_8888 (Bayer sensor =
     * the only source). Shader handles crop + rescale per push-constant. */
    unsigned streamW = srcBuf.stream->width;
    unsigned streamH = srcBuf.stream->height;

    sp<GraphicBuffer> gb = new GraphicBuffer(streamW, streamH,
        HAL_PIXEL_FORMAT_RGBA_8888,
        GRALLOC_USAGE_HW_TEXTURE | GRALLOC_USAGE_HW_COMPOSER,
        streamW, const_cast<native_handle_t *>(*srcBuf.buffer),
        false);
    int zcReleaseFd = -1;
    bool zcOk = false;
    BENCHMARK_SECTION("Raw->RGBA") {
        CropRect crop = { ctx.cropX, ctx.cropY, ctx.cropW, ctx.cropH };
        zcOk = mDeps.isp->processToGralloc(ctx.frameBuf, gb->getNativeBuffer(),
                                            ctx.resW, ctx.resH,
                                            streamW, streamH,
                                            ctx.pixFmt,
                                            -1, &zcReleaseFd, ctx.frameSlotIdx,
                                            crop);
    }
    if (!zcOk)
        return false;

    state->releaseFd = zcReleaseFd;
    if (mDeps.af && mDeps.af->isSweeping() && !*sharedRgba) {
        /* AF sharpness metric needs CPU-readable pixels. Lock the
         * first eligible output per frame so subsequent outputs skip
         * the ~25ms blocklinear detile. Replaced by ISP statistics
         * in Tier 3. */
        const Rect rect((int)streamW, (int)streamH);
        uint8_t *buf = NULL;
        GraphicBufferMapper::get().lock(*srcBuf.buffer,
            GRALLOC_USAGE_SW_READ_OFTEN, rect, (void **)&buf);
        *sharedRgba = buf;
    } else {
        state->needsFinalUnlock = false;
    }
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
                                      uint8_t **sharedRgba) {
    state->needsFinalUnlock = true;
    state->releaseFd        = -1;

    status_t e = waitAcquireFence(srcBuf, frameNumber);
    if (e != NO_ERROR)
        return e;

    switch (srcBuf.stream->format) {
        case HAL_PIXEL_FORMAT_RGBA_8888:
            /* Zero-copy is the only RGBA path. processToGralloc must
             * succeed on Bayer hardware; a false return here is a
             * hardware / driver failure, not a fall-through trigger. */
            if (!tryZeroCopy(srcBuf, ctx, state, sharedRgba)) {
                ALOGE("buffer %p  frame %-4u  zero-copy failed",
                      srcBuf.buffer, frameNumber);
                return NO_INIT;
            }
            return NO_ERROR;
        case HAL_PIXEL_FORMAT_BLOB: {
            uint8_t *buf = NULL;
            e = lockSwWrite(srcBuf, frameNumber, &buf);
            if (e != NO_ERROR)
                return e;
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
