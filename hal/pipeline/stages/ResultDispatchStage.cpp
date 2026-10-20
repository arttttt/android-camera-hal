#include "ResultDispatchStage.h"

#include <stddef.h>
#include <stdint.h>
#include <unistd.h>

#include <utils/Errors.h>
#include <utils/Log.h>
#include <utils/Timers.h>
#include <utils/Vector.h>
#include <ui/GraphicBufferMapper.h>

#include "PipelineContext.h"
#include "BayerSource.h"
#include "3a/AutoFocusController.h"
#include "metadata/CameraStaticMetadata.h"
#include "metadata/ResultMetadataBuilder.h"
#include "pipeline/PartialEmitter.h"
#include "sensor/SensorConfig.h"

#undef  LOG_TAG
#define LOG_TAG "Cam-ResultDispatchStage"

namespace android {

ResultDispatchStage::ResultDispatchStage(const Deps &d) : deps(d) {}

void ResultDispatchStage::returnBuffersInError(PipelineContext &ctx) {
    /* The per-output vectors are filled by DemosaicBlitStage; a frame
     * that failed or was cancelled before it has them empty. */
    const size_t n = ctx.request.outputBuffers.size();
    Vector<camera3_stream_buffer> buffers;
    buffers.setCapacity(n);
    for (size_t i = 0; i < n; ++i) {
        CaptureRequest::Buffer &ob = ctx.request.outputBuffers[i];
        if (i < ctx.outputNeedsFinalUnlock.size() && ctx.outputNeedsFinalUnlock[i]) {
            GraphicBufferMapper::get().unlock(*ob.buffer);
        }
        /* camera3.h: the release fence of a failed buffer is the
         * acquire fence the framework gave us if we still hold it,
         * else -1. An output the GPU or the VIC was already told to
         * write carries their release fence instead, so the buffer is
         * not handed on while it is still being written. */
        int releaseFence = ob.acquireFence.release();
        if (i < ctx.outputReleaseFences.size() && ctx.outputReleaseFences[i] >= 0) {
            if (releaseFence >= 0) ::close(releaseFence);
            releaseFence = ctx.outputReleaseFences[i];
            ctx.outputReleaseFences[i] = -1;
        }
        camera3_stream_buffer sb;
        sb.stream        = ob.stream;
        sb.buffer        = ob.buffer;
        sb.status        = CAMERA3_BUFFER_STATUS_ERROR;
        sb.acquire_fence = -1;
        sb.release_fence = releaseFence;
        buffers.push_back(sb);
    }
    if (buffers.isEmpty()) return;

    if (deps.emitter) {
        /* After ERROR_REQUEST only buffers in error may follow: no
         * metadata, partial_result 0. */
        deps.emitter->emit(ctx.request.frameNumber, 0, nullptr,
                           buffers.size(), buffers.array());
    } else {
        for (const camera3_stream_buffer &sb : buffers) {
            if (sb.release_fence >= 0) ::close(sb.release_fence);
        }
    }
}

void ResultDispatchStage::process(PipelineContext &ctx) {
    const camera3_callback_ops_t *ops = *deps.callbackOps;

    /* Defensive close of any lingering submit sync_fds. PipelineThread's
     * reap path drains them before handing the ctx over, so in normal
     * operation this vector is empty; on stop-drain timeouts it may
     * hold unsignalled fds that ::close() simply releases. */
    for (int fd : ctx.pendingFenceFds) {
        if (fd >= 0) ::close(fd);
    }
    ctx.pendingFenceFds.clear();

    /* A failed or cancelled frame is one complete failure to Camera3:
     * the frontend's word for both is ERROR_REQUEST. */
    if (!ctx.status.ok()) {
        if (ops) {
            camera3_notify_msg_t msg;
            msg.type = CAMERA3_MSG_ERROR;
            msg.message.error.frame_number = ctx.request.frameNumber;
            msg.message.error.error_stream = nullptr;
            msg.message.error.error_code   = CAMERA3_MSG_ERROR_REQUEST;
            ops->notify(ops, &msg);
        }
        returnBuffersInError(ctx);
        if (ctx.bayerFrame) deps.bayerSource->releaseFrame(ctx.bayerFrame);
        return;
    }

    Vector<camera3_stream_buffer> buffers;
    buffers.setCapacity(ctx.request.outputBuffers.size());
    for (size_t i = 0; i < ctx.request.outputBuffers.size(); ++i) {
        const CaptureRequest::Buffer &ob = ctx.request.outputBuffers[i];
        if (ctx.outputNeedsFinalUnlock[i]) {
            GraphicBufferMapper::get().unlock(*ob.buffer);
        }
        camera3_stream_buffer sb;
        sb.stream        = ob.stream;
        sb.buffer        = ob.buffer;
        sb.status        = ctx.outputStatuses[i];
        sb.acquire_fence = -1;
        sb.release_fence = ctx.outputReleaseFences[i];
        buffers.push_back(sb);
    }

    deps.bayerSource->releaseFrame(ctx.bayerFrame);

    ResultMetadataBuilder::FrameState fs;
    fs.timestampNs       = ctx.tShutter ? ctx.tShutter : ctx.tAccepted;
    fs.frameNumber       = ctx.request.frameNumber;
    fs.appliedExposureUs = ctx.appliedExposureUs;
    fs.appliedGain       = ctx.appliedGain;
    fs.aeConverged       = ctx.aeConverged;
    fs.af.afMode       = ANDROID_CONTROL_AF_MODE_OFF;
    fs.af.afState      = ANDROID_CONTROL_AF_STATE_INACTIVE;
    fs.af.focusDiopter = 0.0f;
    if (deps.af) fs.af = deps.af->report();
    ResultMetadataBuilder::build(ctx.request.settings, fs, *deps.sensorCfg);

    const camera_metadata_t *result = ctx.request.settings.getAndLock();
    if (deps.emitter) {
        /* Final partial of the frame: carries the buffer pointers
         * and (until the per-controller partial-emit step lands)
         * the full metadata blob. Counter = `kPartialResultCount`
         * signals "this is the last partial for this frame" to the
         * framework; intermediate partials (1..count-1) will be
         * issued from the IPA tick once 3A is split into separate
         * controllers, each with its own metadata subset. */
        deps.emitter->emit(ctx.request.frameNumber,
                           kPartialResultCount,
                           result,
                           buffers.size(),
                           buffers.array());
    }
    ctx.request.settings.unlock(result);

    ctx.tResultSent = systemTime();
    int64_t wait  = (ctx.tBayerDq    - ctx.tShutter) / 1000000;
    int64_t post  = (ctx.tResultSent - ctx.tBayerDq) / 1000000;
    int64_t total = (ctx.tResultSent - ctx.tShutter) / 1000000;
    ALOGD("PERF: wait=%lldms post=%lldms total=%lldms f=%u",
          (long long)wait, (long long)post, (long long)total,
          ctx.request.frameNumber);
}

} /* namespace android */
