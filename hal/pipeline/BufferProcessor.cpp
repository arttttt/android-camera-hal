#define LOG_TAG "Cam-BufProc"

#include "BufferProcessor.h"

#include <string.h>
#include <linux/videodev2.h>
#include <libyuv/scale_argb.h>

#include <hardware/gralloc.h>
#include <ui/Rect.h>
#include <ui/GraphicBuffer.h>
#include <ui/GraphicBufferMapper.h>
#include <ui/Fence.h>
#include <utils/Log.h>

#include "IspPipeline.h"
#include "ImageConverter.h"
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

status_t BufferProcessor::processOne(const camera3_stream_buffer &srcBuf,
                                      const FrameContext &ctx,
                                      const CameraMetadata &cm,
                                      uint32_t frameNumber,
                                      OutputState *state,
                                      uint8_t **sharedRgba) {
    state->needsFinalUnlock = true;
    state->releaseFd        = -1;

    uint8_t *buf = NULL;

    /* Acquire fence — wait for the framework/consumer to release the
     * buffer to us for writing. */
    {
        sp<Fence> acquireFence = new Fence(srcBuf.acquire_fence);
        status_t e = acquireFence->wait(kAcquireFenceTimeoutMs);
        if (e == TIMED_OUT) {
            ALOGE("buffer %p  frame %-4u  Wait on acquire fence timed out",
                  srcBuf.buffer, frameNumber);
        }
        if (e != NO_ERROR)
            return NO_INIT;
    }

    /* Zero-copy eligibility: RGBA preview stream, matching resolution,
     * no zoom, Bayer input. Attempt before taking SW_WRITE_OFTEN lock
     * so we skip the ~25ms blocklinear→staging detile when the GPU can
     * write directly into gralloc. */
    unsigned streamW = srcBuf.stream->width;
    unsigned streamH = srcBuf.stream->height;
    bool zcEligible = (srcBuf.stream->format == HAL_PIXEL_FORMAT_RGBA_8888 &&
                       !*sharedRgba && !ctx.needZoom &&
                       ctx.resW == streamW && ctx.resH == streamH &&
                       ctx.pixFmt != V4L2_PIX_FMT_UYVY &&
                       ctx.pixFmt != V4L2_PIX_FMT_YUYV);
    if (zcEligible) {
        sp<GraphicBuffer> gb = new GraphicBuffer(streamW, streamH,
            HAL_PIXEL_FORMAT_RGBA_8888,
            GRALLOC_USAGE_HW_TEXTURE | GRALLOC_USAGE_HW_COMPOSER,
            streamW, const_cast<native_handle_t *>(*srcBuf.buffer),
            false);
        int zcReleaseFd = -1;
        bool zcOk = false;
        BENCHMARK_SECTION("Raw->RGBA") {
            zcOk = mDeps.isp->processToGralloc(ctx.frameBuf, gb->getNativeBuffer(),
                                                streamW, streamH, ctx.pixFmt,
                                                -1, &zcReleaseFd, ctx.frameSlotIdx);
        }
        if (zcOk) {
            state->releaseFd = zcReleaseFd;
            if (mDeps.af && mDeps.af->isSweeping()) {
                /* AF sharpness metric needs CPU-readable pixels. This
                 * lock blocks until the GPU finishes (gralloc internally
                 * syncs), but AF sweeps are infrequent. */
                const Rect rect((int)streamW, (int)streamH);
                GraphicBufferMapper::get().lock(*srcBuf.buffer,
                    GRALLOC_USAGE_SW_READ_OFTEN, rect, (void **)&buf);
                *sharedRgba = buf;
            } else {
                state->needsFinalUnlock = false;
            }
            return NO_ERROR;
        }
        /* Fall through to CPU path on zero-copy failure. */
    }

    {
        const Rect rect((int)streamW, (int)streamH);
        status_t e = GraphicBufferMapper::get().lock(*srcBuf.buffer,
            GRALLOC_USAGE_SW_WRITE_OFTEN, rect, (void **)&buf);
        if (e != NO_ERROR) {
            ALOGE("buffer %p  frame %-4u  lock failed", srcBuf.buffer, frameNumber);
            return NO_INIT;
        }
    }

    switch (srcBuf.stream->format) {
        case HAL_PIXEL_FORMAT_RGBA_8888: {
            if (!*sharedRgba) {
                BENCHMARK_SECTION("Raw->RGBA") {
                    if (ctx.pixFmt == V4L2_PIX_FMT_UYVY) {
                        mDeps.converter->UYVYToRGBA(ctx.frameBuf,
                            ctx.needZoom ? ctx.rgbaScratch : buf,
                            ctx.resW, ctx.resH);
                        *sharedRgba = ctx.needZoom ? ctx.rgbaScratch : buf;
                    } else if (ctx.pixFmt == V4L2_PIX_FMT_YUYV) {
                        mDeps.converter->YUY2ToRGBA(ctx.frameBuf,
                            ctx.needZoom ? ctx.rgbaScratch : buf,
                            ctx.resW, ctx.resH);
                        *sharedRgba = ctx.needZoom ? ctx.rgbaScratch : buf;
                    } else {
                        /* Zoom or size mismatch — CPU readback path.
                         * processSync honours srcInputSlot so DMABUF
                         * capture (frameBuf == NULL) still resolves. */
                        uint8_t *convDst = ctx.needZoom ? ctx.rgbaScratch : buf;
                        if (!ctx.needZoom && (ctx.resW != streamW || ctx.resH != streamH))
                            convDst = ctx.rgbaScratch;
                        mDeps.isp->processSync(ctx.frameBuf, convDst,
                                                ctx.resW, ctx.resH, ctx.pixFmt,
                                                ctx.frameSlotIdx);
                        *sharedRgba = convDst;
                    }
                }
            }

            BENCHMARK_SECTION("Zoom/Copy") {
                if (ctx.needZoom) {
                    const uint8_t *cropSrc = *sharedRgba
                        + (ctx.cropY * (int)ctx.resW + ctx.cropX) * 4;
                    libyuv::ARGBScale(cropSrc, ctx.resW * 4, ctx.cropW, ctx.cropH,
                                      buf, streamW * 4, streamW, streamH,
                                      libyuv::kFilterBilinear);
                } else if (*sharedRgba != buf) {
                    unsigned copyW = (streamW < ctx.resW) ? streamW : ctx.resW;
                    unsigned copyH = (streamH < ctx.resH) ? streamH : ctx.resH;
                    for (unsigned y = 0; y < copyH; y++)
                        memcpy(buf + y * streamW * 4,
                               *sharedRgba + y * ctx.resW * 4,
                               copyW * 4);
                }
            }
            break;
        }
        case HAL_PIXEL_FORMAT_BLOB: {
            BENCHMARK_SECTION("YUV->JPEG") {
                JpegSource jsrc;
                jsrc.frameBuf     = ctx.frameBuf;
                jsrc.srcInputSlot = ctx.frameSlotIdx;
                jsrc.pixFmt       = ctx.pixFmt;
                jsrc.width        = ctx.resW;
                jsrc.height       = ctx.resH;
                mDeps.jpeg->encode(buf, ctx.jpegBufferSize, jsrc, cm, ctx.rgbaScratch);
            }
            break;
        }
        default:
            ALOGE("Unknown pixel format %d in buffer %p (stream %p), ignoring",
                  srcBuf.stream->format, srcBuf.buffer, srcBuf.stream);
    }

    return NO_ERROR;
}

}; /* namespace android */
