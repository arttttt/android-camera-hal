#define LOG_TAG "Cam-BufProc"

#include "BufferProcessor.h"

#include <errno.h>
#include <poll.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <utils/Log.h>
#include <utils/Timers.h>
#include <libyuv.h>

#include "IspPipeline.h"
#include "PostProcessor.h"
#include "sensor/SensorConfig.h"
#include "DbgUtils.h"
#include "mochacamera/ports/GrallocPort.h"

namespace android {

using mocha::camera::BufferStatus;
using mocha::camera::FrameSettings;
using mocha::camera::GrallocPort;
using mocha::camera::PixelFormat;

namespace {

/* Timeout for acquire_fence wait — preserves the pre-existing magic
 * number; revisit when fence semantics are formalised. */
constexpr int kAcquireFenceTimeoutMs = 1000;

/* The camera3 BLOB trailer, written at the very end of the buffer: the
 * framework finds the JPEG's size there. Its layout is camera3.h's
 * camera3_jpeg_blob, which the core does not include. */
struct JpegBlobTrailer {
    uint16_t jpegBlobId;
    uint32_t jpegSize;
};
constexpr uint16_t kJpegBlobId = 0x00FF;
static_assert(sizeof(JpegBlobTrailer) == 8, "camera3_jpeg_blob is 8 bytes");

/* The part of the frame's crop an output samples: its own aspect,
 * centred (cropForOutput). */
CropRect outputCrop(const BufferProcessor::Output &out,
                    const BufferProcessor::FrameContext &ctx) {
    return cropForOutput(CropRect{ctx.cropX, ctx.cropY, ctx.cropW, ctx.cropH},
                         out.stream.width, out.stream.height);
}

} /* namespace */

BufferProcessor::BufferProcessor(const Deps &deps)
    : mDeps(deps) {
}

status_t BufferProcessor::waitAcquireFence(Output &out, uint32_t frameNumber) {
    /* Acquire fence signals when the gralloc buffer is safe for the
     * HAL to write. On a backed-up compositor this is the main way the
     * framework back-pressures us — measuring it surfaces the case
     * where "GPU isn't the bottleneck, the display is". Only log when
     * the wait is meaningful (>5 ms) to keep the steady-state log
     * readable. */
    UniqueFd fence = std::move(out.acquireFence);
    if (!fence.valid())
        return NO_ERROR;
    int64_t t0 = systemTime();
    struct pollfd p = { fence.get(), POLLIN, 0 };
    int r;
    do {
        r = ::poll(&p, 1, kAcquireFenceTimeoutMs);
    } while (r < 0 && errno == EINTR);
    int64_t waitMs = (systemTime() - t0) / 1000000;
    if (waitMs > 5) {
        ALOGD("acquire_fence wait %lldms  frame %u  buffer %p",
              (long long)waitMs, frameNumber, out.buffer);
    }
    if (r == 0) {
        ALOGE("buffer %p  frame %-4u  Wait on acquire fence timed out",
              out.buffer, frameNumber);
        return NO_INIT;
    }
    if (r < 0 || (p.revents & (POLLERR | POLLNVAL)))
        return NO_INIT;
    return NO_ERROR;
}

bool BufferProcessor::tryZeroCopy(Output &out, const FrameContext &ctx) {
    /* The caller guarantees this stream is RGBA_8888 (Bayer sensor =
     * the only source). The shader handles crop + rescale per push-constant.
     * The acquire fence (sync_fd) gets imported as a binary VkSemaphore so
     * the eventual submit GPU-waits on framework readiness without
     * blocking this thread. */
    bool zcOk = false;
    BENCHMARK_SECTION("Raw->RGBA") {
        zcOk = mDeps.recorder->blitToGralloc(out.buffer,
                                         out.stream.width, out.stream.height,
                                         outputCrop(out, ctx),
                                         out.acquireFence.release(),
                                         &out.releaseFence);
    }
    return zcOk;
}

status_t BufferProcessor::recordYuvOutput(Output &out,
                                            const FrameContext &ctx,
                                            uint32_t frameNumber) {
    /* CPU-wait on the framework acquire_fence here so finalizeYuvOutput's
     * lock (post-fence-reap) doesn't have to block. The wait is a few ms
     * in steady state; the GPU encode dispatch we record next runs in
     * parallel with subsequent frames. */
    status_t e = waitAcquireFence(out, frameNumber);
    if (e != NO_ERROR)
        return e;

    int releaseFd = -1;
    bool ok = false;
    BENCHMARK_SECTION("Raw->NV12") {
        ok = mDeps.recorder->blitToYuv(out.buffer, out.stream.width, out.stream.height,
                                  outputCrop(out, ctx), -1, &releaseFd);
    }
    if (!ok) {
        ALOGE("buffer %p  frame %-4u  blitToYuv failed", out.buffer, frameNumber);
        return NO_INIT;
    }
    return NO_ERROR;
}

void BufferProcessor::finalizeYuvOutput(Output &out, uint32_t frameNumber) {
    unsigned w = out.stream.width;
    unsigned h = out.stream.height;

    mDeps.yuv->invalidateYuvForCpu();
    const uint8_t *nv12 = mDeps.yuv->yuvHostBuffer();
    if (!nv12) {
        ALOGE("buffer %p  frame %-4u  yuvHostBuffer null at finalize",
              out.buffer, frameNumber);
        return;
    }
    const uint8_t *srcY  = nv12;
    const uint8_t *srcUv = nv12 + (size_t)w * h;

    GrallocPort::Ycbcr dst;
    if (!mDeps.gralloc->lockYcbcr(out.buffer, out.stream, &dst)) {
        ALOGE("buffer %p  frame %-4u  lockYCbCr failed", out.buffer, frameNumber);
        return;
    }

    BENCHMARK_SECTION("NV12->gralloc") {
        if (dst.chromaStep == 2 && dst.cb + 1 == dst.cr) {
            libyuv::CopyPlane(srcY,  (int)w, dst.y,  (int)dst.yStride,
                              (int)w, (int)h);
            libyuv::CopyPlane(srcUv, (int)w, dst.cb, (int)dst.cStride,
                              (int)w, (int)h / 2);
        } else if (dst.chromaStep == 1) {
            libyuv::NV12ToI420(srcY, (int)w, srcUv, (int)w,
                               dst.y,  (int)dst.yStride,
                               dst.cb, (int)dst.cStride,
                               dst.cr, (int)dst.cStride,
                               (int)w, (int)h);
        } else {
            ALOGE("buffer %p  frame %-4u  YUV layout not supported "
                  "(chroma_step=%zu, cb=%p, cr=%p)",
                  out.buffer, frameNumber,
                  dst.chromaStep, dst.cb, dst.cr);
        }
    }

    mDeps.gralloc->unlock(out.buffer);
}

void BufferProcessor::finalizeCpuOutputs(Frame &frame) {
    if (!frame.status.ok()) return;
    for (Output &out : frame.outputs) {
        if (out.status != BufferStatus::Ok) continue;
        /* BLOB encode runs on the JPEG engine — handled out of band. */
        if (out.stream.format == PixelFormat::Ycbcr420)
            finalizeYuvOutput(out, frame.frameNumber);
    }
}

void BufferProcessor::releaseJpegSnapshots(Frame &frame) {
    for (size_t i = 0; i < frame.outputs.size(); ++i)
        releaseJpegSnapshot(frame, i);
}

void BufferProcessor::releaseJpegSnapshot(Frame &frame, size_t i) {
    if (i >= frame.outputs.size()) return;
    Output &out = frame.outputs[i];
    if (out.jpeg.ringSlot >= 0) {
        mDeps.jpegRing->releaseJpegSnapshot(out.jpeg);
        out.jpeg.ringSlot = -1;
    }
}

status_t BufferProcessor::recordBlobOutput(Output &out, const FrameContext &ctx,
                                           uint32_t frameNumber) {
    /* CPU-wait acquire_fence here so finalizeBlobOutput's lock runs
     * without blocking. Same pattern as YUV. */
    status_t e = waitAcquireFence(out, frameNumber);
    if (e != NO_ERROR)
        return e;

    if (!mDeps.recorder->blitToJpegCpu(outputCrop(out, ctx), &out.jpeg)) {
        ALOGE("buffer %p  frame %-4u  blitToJpegCpu failed", out.buffer, frameNumber);
        return NO_INIT;
    }
    ALOGD("BLOB record: frame=%u ringSlot=%d %ux%u",
          frameNumber, out.jpeg.ringSlot, out.jpeg.width, out.jpeg.height);
    return NO_ERROR;
}

/* What the picture's EXIF says: what its frame ran on, the request's
 * location, the device and the lens; dated now, by the wall clock in
 * the local zone. */
ExifInfo BufferProcessor::exifFor(const Frame &frame) const {
    ExifInfo e;
    struct timespec now;
    clock_gettime(CLOCK_REALTIME, &now);
    const time_t sec = now.tv_sec;
    struct tm local;
    localtime_r(&sec, &local);
    e.timeSec      = now.tv_sec;
    e.subSecMs     = (int32_t)(now.tv_nsec / 1000000);
    e.utcOffsetMin = (int32_t)(local.tm_gmtoff / 60);

    e.make  = mDeps.make;
    e.model = mDeps.model;
    if (mDeps.fNumber > 0)       e.fNumber       = mDeps.fNumber;
    if (mDeps.focalLengthMm > 0) e.focalLengthMm = mDeps.focalLengthMm;

    const android::DelayedControls::Batch &ran = frame.inEffect;
    if (ran.has[android::DelayedControls::EXPOSURE])
        e.exposureNs = (int64_t)ran.val[android::DelayedControls::EXPOSURE] * 1000;
    if (ran.has[android::DelayedControls::GAIN] && mDeps.sensorCfg)
        e.iso = mDeps.sensorCfg->gainToIso(ran.val[android::DelayedControls::GAIN]);
    e.manualExposure     = frame.settings.ae.mode == mocha::camera::AeMode::Off;
    e.manualWhiteBalance = frame.settings.awb.mode == mocha::camera::AwbMode::Off;

    if (const auto &g = frame.settings.jpeg.gps) {
        ExifInfo::Gps gps;
        gps.latitude         = g->latitude;
        gps.longitude        = g->longitude;
        gps.altitude         = g->altitude;
        gps.timestampSec     = g->timestampSec;
        gps.processingMethod = g->processingMethod;
        e.gps = std::move(gps);
    }
    return e;
}

bool BufferProcessor::finalizeBlobOutput(Frame &frame, size_t outputIndex) {
    Output &out = frame.outputs[outputIndex];
    const FrameSettings::Jpeg &jpeg = frame.settings.jpeg;
    const uint32_t frameNumber = frame.frameNumber;
    const JpegSnapshot snap = out.jpeg;
    if (snap.ringSlot < 0) return false;
    /* The snapshot is used up here whatever happens below. */
    out.jpeg.ringSlot = -1;

    const size_t blobCapacity = mDeps.jpegBufferSize ? *mDeps.jpegBufferSize : 0;
    if (blobCapacity <= sizeof(JpegBlobTrailer)) {
        ALOGE("buffer %p  frame %-4u  BLOB capacity %zu too small",
              out.buffer, frameNumber, blobCapacity);
        mDeps.jpegRing->releaseJpegSnapshot(snap);
        return false;
    }
    const size_t maxImageSize = blobCapacity - sizeof(JpegBlobTrailer);

    uint8_t *dst = NULL;
    if (!mDeps.gralloc->lock(out.buffer, out.stream, &dst)) {
        ALOGE("buffer %p  frame %-4u  BLOB lock failed", out.buffer, frameNumber);
        mDeps.jpegRing->releaseJpegSnapshot(snap);
        return false;
    }

    mDeps.jpegRing->invalidateJpegSnapshot(snap);
    size_t written = 0;
    int64_t t0 = systemTime();
    bool ok = mDeps.jpeg->process(snap, out.stream.width, out.stream.height,
                                  dst, maxImageSize, jpeg, exifFor(frame), &written);
    int64_t encMs = (systemTime() - t0) / 1000000;
    if (ok) {
        JpegBlobTrailer trailer;
        trailer.jpegBlobId = kJpegBlobId;
        trailer.jpegSize   = (uint32_t)written;
        memcpy(dst + maxImageSize, &trailer, sizeof(trailer));
        ALOGD("BLOB finalize: frame=%u ringSlot=%d encode=%lldms size=%zu",
              frameNumber, snap.ringSlot, (long long)encMs, written);
    } else {
        ALOGE("buffer %p  frame %-4u  JPEG encode failed", out.buffer, frameNumber);
    }

    mDeps.gralloc->unlock(out.buffer);
    mDeps.jpegRing->releaseJpegSnapshot(snap);
    return ok;
}

status_t BufferProcessor::processOne(Output &out, const FrameContext &ctx,
                                      uint32_t frameNumber) {
    out.releaseFence = -1;
    out.jpeg = JpegSnapshot{nullptr, 0, 0, 0, -1};

    switch (out.stream.format) {
        case PixelFormat::Rgba8888:
            /* RGBA zero-copy: GPU writes directly into the gralloc via the
             * blit render pass, framework acquire_fence is imported as a
             * VkSemaphore on the submit, release_fence is exported per
             * output by endFrame. The gralloc is never CPU-locked. */
            if (!tryZeroCopy(out, ctx)) {
                ALOGE("buffer %p  frame %-4u  zero-copy failed",
                      out.buffer, frameNumber);
                return NO_INIT;
            }
            return NO_ERROR;
        case PixelFormat::Ycbcr420:
            /* GPU NV12 encode now, libyuv repack into the (pitch-
             * linear) gralloc in finalizeCpuOutputs after the fence
             * reaps. The HW VIC (libnvblit) was tried for this and gave
             * tile-scrambled output in every surface combination; see
             * docs/vic-encoder-path.md. */
            return recordYuvOutput(out, ctx, frameNumber);
        case PixelFormat::Blob:
            /* GPU records vkCmdCopyImageToBuffer(scratch -> jpeg ring slot)
             * now; libjpeg encode runs on the JPEG engine after the
             * frame's fence is reaped; the encode does its own lock and
             * unlock. */
            return recordBlobOutput(out, ctx, frameNumber);
    }
    return BAD_VALUE;
}

}; /* namespace android */
