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
#include "metadata/ResultMetadataBuilder.h"
#include "sensor/SensorConfig.h"

#define LOG_TAG "Cam-ResultDispatchStage"

namespace android {

ResultDispatchStage::ResultDispatchStage(const Deps &d) : deps(d) {}

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

    if (ctx.errorCode) {
        if (ops) {
            camera3_notify_msg_t msg;
            msg.type = CAMERA3_MSG_ERROR;
            msg.message.error.frame_number = ctx.request.frameNumber;
            msg.message.error.error_stream = nullptr;
            msg.message.error.error_code   = CAMERA3_MSG_ERROR_REQUEST;
            ops->notify(ops, &msg);
        }
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
    fs.af.afMode       = ANDROID_CONTROL_AF_MODE_OFF;
    fs.af.afState      = ANDROID_CONTROL_AF_STATE_INACTIVE;
    fs.af.focusDiopter = 0.0f;
    if (deps.af) fs.af = deps.af->report();
    ResultMetadataBuilder::build(ctx.request.settings, fs, *deps.sensorCfg);

    const camera_metadata_t *result = ctx.request.settings.getAndLock();
    if (ops) {
        camera3_capture_result cr;
        cr.frame_number       = ctx.request.frameNumber;
        cr.result             = result;
        cr.num_output_buffers = buffers.size();
        cr.output_buffers     = buffers.array();
        cr.input_buffer       = nullptr;
        cr.partial_result     = 0;
        ops->process_capture_result(ops, &cr);
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
