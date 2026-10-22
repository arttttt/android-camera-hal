#include "ResultDispatchStage.h"

#include <stddef.h>
#include <stdint.h>
#include <unistd.h>

#include <vector>

#include <utils/Log.h>
#include <utils/Timers.h>

#include "Frame.h"
#include "3a/AutoFocusController.h"
#include "3a/ThreeAReport.h"
#include "mochacamera/ports/ResultSink.h"
#include "sensor/SensorConfig.h"

#undef  LOG_TAG
#define LOG_TAG "Cam-ResultDispatchStage"

namespace android {

using mocha::camera::BufferReturn;
using mocha::camera::BufferStatus;
using mocha::camera::CoreFrameResult;
using mocha::camera::Frame;
using mocha::camera::FrameOutcome;

ResultDispatchStage::ResultDispatchStage(const Deps &d) : deps(d) {}

void ResultDispatchStage::process(Frame &frame) {
    const bool completed = frame.status.ok();

    std::vector<BufferReturn> buffers;
    buffers.reserve(frame.outputs.size());
    for (Frame::Output &out : frame.outputs) {
        BufferReturn b;
        b.stream = out.stream.id;
        b.buffer = out.buffer;
        b.status = completed ? out.status : BufferStatus::Error;
        /* The release fence of a buffer is the one the GPU was told to
         * signal for it; a buffer nothing was written to goes back with
         * the acquire fence it came with, if the HAL still holds it. */
        if (out.releaseFence >= 0) {
            b.releaseFence   = out.releaseFence;
            out.releaseFence = -1;
        } else {
            b.releaseFence = out.acquireFence.release();
        }
        buffers.push_back(b);
    }

    CoreFrameResult result;
    result.outcome     = completed ? FrameOutcome::Completed : FrameOutcome::Failed;
    result.timestampNs = frame.tShutter ? frame.tShutter : frame.tAccepted;
    if (completed) {
        AutoFocusController::Report af;
        af.afState      = mocha::camera::AfState::Inactive;
        af.afMode       = mocha::camera::AfMode::Off;
        af.focusDiopter = 0.0f;
        if (deps.af) af = deps.af->report();
        result.a3 = describe3A(frame.settings, af,
                               {frame.aeConverged, frame.awbConverged},
                               frame.inEffect, *deps.sensorCfg);
    }

    deps.sink->frame(frame.frameNumber, result, buffers);

    if (completed) {
        frame.tResultSent = systemTime();
        int64_t wait  = (frame.tBayerDq    - frame.tShutter) / 1000000;
        int64_t post  = (frame.tResultSent - frame.tBayerDq) / 1000000;
        int64_t total = (frame.tResultSent - frame.tShutter) / 1000000;
        ALOGD("PERF: wait=%lldms post=%lldms total=%lldms f=%u",
              (long long)wait, (long long)post, (long long)total,
              frame.frameNumber);
    }
}

} /* namespace android */
