/*
 * Copyright (C) 2026 Artem Bambalov
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CORE_PIPELINE_FRAME_H
#define CORE_PIPELINE_FRAME_H

#include <atomic>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "BayerLease.h"
#include "CaptureRequest.h"
#include "EventFd.h"
#include "FrameStatus.h"
#include "JpegSnapshot.h"

namespace android { class IspPipeline; }

namespace mocha::camera {

/* One capture request, from the moment the frontend accepts it until
 * its result is handed back.
 *
 * Shared, not passed around by raw pointer: the registry holds it for
 * flush and close, the result engine holds it until it has handed the
 * result back, and every job working on it holds it through a
 * FrameToken -- so no job can outlive the frame it works on.
 *
 * Fields are written by one engine at a time, in the order the frame
 * travels (capture, demosaic, finish, JPEG, result); handing a job to
 * the next engine's queue orders each write before the next engine's
 * reads. The exceptions say so.
 *
 * All time fields are CLOCK_MONOTONIC nanoseconds (systemTime()). */
struct Frame {
    uint32_t                frameNumber = 0;
    android::CaptureRequest request;

    /* Set from the request on acceptance: a snapshot (still capture,
     * video snapshot or ZSL intent, or a BLOB output) is never given up
     * by the AE mode switch. */
    bool stillCapture = false;

    /* Set by the capture engine once the frame has its Bayer. Read
     * without a lock by the AE mode switch, which only gives up frames
     * still waiting for one. */
    std::atomic<bool> hasBayer{false};

    /* The capture slot the frame was taken from. Let go by the demosaic
     * engine once the GPU has read it, or with the frame. */
    BayerLease bayer;
    int cropX = 0;
    int cropY = 0;
    int cropW = 0;
    int cropH = 0;

    /* Filled by the demosaic. Indexed in lockstep with
     * request.outputBuffers -- element i is output i. */
    std::vector<int>  outputReleaseFences;
    /* Written by the JPEG engine for a BLOB it could not encode; read
     * by the result engine only once every token is gone. */
    std::vector<int>  outputStatuses;
    /* JPEG snapshots for BLOB outputs; ringSlot < 0 for any other
     * output, or when the snapshot could not be taken. Whoever uses one
     * up -- the encode, or the finish engine giving the frame's up --
     * sets its ringSlot to -1; any still held when the frame goes are
     * given back to `jpegRing` then, so a frame dropped on its way (an
     * engine stopping with it queued) cannot keep a slot of the ring. */
    std::vector<android::JpegSnapshot> outputJpegSnapshots;
    android::IspPipeline              *jpegRing = nullptr;

    /* What this request wrote to the sensor. It takes effect a control
     * delay later; result metadata reports what was in effect from
     * DelayedControls instead. */
    int32_t appliedExposureUs = 0;
    int32_t appliedGain       = 0;
    bool    aeConverged       = false;

    /* Timestamps for PERF diagnostics. */
    int64_t tAccepted   = 0;
    int64_t tShutter    = 0;
    int64_t tBayerDq    = 0;
    int64_t tResultSent = 0;

    /* How the frame ends. Anything but Ok means it is being aborted:
     * engines skip their work and the result engine reports the
     * request as failed. */
    android::FrameStatus status;

    /* Jobs still working on the frame, counted by FrameToken; the
     * result engine hands the frame back once this reaches zero, and
     * `settled` (the result engine's wake) is notified when it does. */
    std::atomic<int>   inPipeline{0};
    android::EventFd  *settled = nullptr;

    Frame() = default;
    ~Frame();
    Frame(const Frame &) = delete;
    Frame &operator=(const Frame &) = delete;
};

using FrameRef = std::shared_ptr<Frame>;

/* A job's hold on a frame it is working on. While any token is alive
 * the frame is in the pipeline and the result engine waits for it; the
 * last token to go -- the job handing the frame on did not pass it, or
 * the work ended -- wakes the result engine. A job hands the frame to
 * the next engine by moving its token into the next job; a job that
 * fans out (one JPEG job per BLOB output) gives each a share().
 *
 * No path loses a frame: a job that is done, refused, cancelled,
 * dropped, or let go by a stopping engine destroys its token, and the
 * frame goes on to its result either way. */
class FrameToken {
public:
    FrameToken() = default;
    explicit FrameToken(FrameRef frame) : frame(std::move(frame)) {
        if (this->frame)
            this->frame->inPipeline.fetch_add(1, std::memory_order_relaxed);
    }

    FrameToken(FrameToken &&) noexcept = default;
    FrameToken &operator=(FrameToken &&other) noexcept {
        if (this != &other) {
            leave();
            frame = std::move(other.frame);
        }
        return *this;
    }
    FrameToken(const FrameToken &) = delete;
    FrameToken &operator=(const FrameToken &) = delete;

    ~FrameToken() { leave(); }

    FrameToken share() const { return FrameToken(frame); }

    explicit operator bool() const { return frame != nullptr; }
    Frame &operator*() const { return *frame; }
    Frame *operator->() const { return frame.get(); }

private:
    void leave() {
        if (!frame)
            return;
        FrameRef f = std::move(frame);
        if (f->inPipeline.fetch_sub(1, std::memory_order_acq_rel) == 1 &&
            f->settled)
            f->settled->notify();
    }

    FrameRef frame;
};

} /* namespace mocha::camera */

#endif /* CORE_PIPELINE_FRAME_H */
