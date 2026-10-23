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

#ifndef CORE_ENGINES_CAPTURE_ENGINE_H
#define CORE_ENGINES_CAPTURE_ENGINE_H

#include <memory>
#include <optional>
#include <variant>
#include <vector>

#include "Engine.h"
#include "EventFd.h"
#include "Frame.h"
#include "PermitPool.h"
#include "UniqueFd.h"
#include "mochacamera/Controls.h"
#include "mochacamera/ports/FrameSource.h"
#include "sensor/FrameBinder.h"

namespace android {
class AutoFocusController;
class DeviceErrorReporter;
class ExposureTarget;
class FrameMap;
class IspInputRing;
class SensorWriter;
struct SensorConfig;
}

namespace mocha::camera {

class IspEngine;
class ResultSink;
class StatsEngine;

/* Where a request meets the sensor frame its settings land on.
 *
 * The capture driver says when each frame starts (V4L2_EVENT_FRAME_SYNC,
 * numbered as its buffer will be). The start goes to the FrameBinder,
 * and what it says to write goes to the sensor (SensorWriter) at once,
 * inside the frame: the sensor's delays count from there, whatever the
 * mode's blanking. Each frame, as it arrives, goes to the binder too:
 * to the request that asked for it, or back to the source. A request, as it
 * arrives, asks the binder for its frame with its settings (what AE or
 * the request wants, requestExposure), and waits for it.
 *
 * Every request is taken at once (maxRunning is not a depth here: a
 * request needs its frame chosen as it arrives, not when the one before
 * it is done), and they move on in the order they came: their frames
 * come in that order, and the shutter goes out in it.
 *
 * A request's job, once its frame has come (a request whose frame is
 * lost fails):
 *   1. the shutter, at the frame's exposure start;
 *   2. a permit for the demosaic engine; without one, wait;
 *   3. the crop, a lease to the stats engine, the frame and its permit
 *      to the demosaic engine.
 * A request cancelled at any point -- flush, the AE mode switch -- ends
 * its job at the next look; wake() makes the look happen now.
 *
 * The thread also owns the V4L2 queue: the node is watched with the
 * jobs, and slots that leases give back are queued here, so V4L2 has
 * one writer.
 *
 * A sensor that stops is told by its frames, not by the requests: a
 * request may wait seconds for its frame behind long exposures. Each
 * frame's start arms a timer for its period, as written, plus
 * kStallSlack; when it fires, the device is reported lost. */
class CaptureEngine {
public:
    struct Deps {
        FrameSource                  *source;
        android::IspInputRing        *inputRing;   /* CPU view of a slot, for stats */
        android::AutoFocusController *af;          /* may be null: AF settings, ROI */
        /* The running mode's frame against the active array; changed
         * only while the engine is stopped. */
        const android::FrameMap      *frameMap;
        /* The fewest patches a side an AE region meters over
         * (SensorTuning::AeParams::minMeteringRoiPatches). */
        int                           meteringMinPatches = 1;
        ResultSink                   *sink;        /* the shutter */
        android::DelayedControls     *controls;
        android::SensorWriter        *writer;
        const android::SensorConfig  *sensorCfg;
        const android::ExposureTarget *exposureTarget; /* may be null: manual only */
        Rational                      evStep;      /* Capability::ae.evStep */
        /* A write goes this long after its frame's start is reported
         * (Session::Config::writeDelayUs); 0: at once. */
        int64_t                       writeDelayNs = 0;
        IspEngine                    *demosaic;
        StatsEngine                  *stats;       /* may be null */
        android::DeviceErrorReporter *deviceError; /* may be null */
    };

    explicit CaptureEngine(const Deps &deps);
    ~CaptureEngine();

    CaptureEngine(const CaptureEngine &) = delete;
    CaptureEngine &operator=(const CaptureEngine &) = delete;

    /* A new stream: frames are numbered from 0, and DelayedControls must
     * have been reset to what configure wrote. */
    bool start();
    void stop();

    /* False once stopped; the token goes, and the frame with it to its
     * result. */
    bool submit(FrameToken frame);

    /* Have the waiting frames look at their status again. */
    void wake() { wakeFd.notify(); }

    /* Frames for the demosaic engine at once: those on the GPU, and one
     * handed over and waiting. */
    static constexpr int kPermits = 2;

    /* Capture slots held besides the requests' own: the one the VI has
     * armed, and the stats cycle's, which can outlive its frame's
     * demosaic. Every other slot is a request's, from its frame's
     * arrival to the GPU's read. */
    static constexpr int kSlotsBesidesRequests = 2;

private:
    struct Job {
        FrameToken                                   frame;
        std::shared_ptr<android::FrameBinder::Ticket> ticket;
        std::optional<Permit>                        permit;
        bool                                         shutterSent = false;
    };
    using Eng = Engine<Job, std::monostate>;

    Eng::Step advance(Job &job);
    void onStarts(const std::vector<FrameStart> &starts);
    void onFrames(std::vector<BayerLease> frames);
    /* The next frame is due within `periodNs` and the slack. */
    void armStall(int64_t periodNs);
    void onStall();
    /* Puts the write held for writeDelayNs on the sensor, if any. */
    void writeHeld();
    void crop(Frame &frame);
    void dispatchStats(Frame &frame);

    /* How late past its period a frame may come before the sensor is
     * taken for stopped: the VI's own wait has its slack on top of the
     * frame, and a frame it gives up on still comes back, damaged. */
    static constexpr int64_t kStallSlackNs = 3000000000LL;

    Deps deps;
    /* Woken by a frame bound, a permit coming back, and wake(). */
    android::EventFd wakeFd;
    PermitPool permits;
    std::unique_ptr<Eng> engine;

    /* The engine thread's own. */
    android::FrameBinder binder;
    android::UniqueFd    stallTimer;
    android::UniqueFd    writeTimer;
    std::optional<android::DelayedControls::Batch> held;
    /* The start the held write is timed from: its frame, and when the
     * driver said it started. */
    uint32_t heldFrame   = 0;
    int64_t  heldStartNs = 0;
};

} /* namespace mocha::camera */

#endif /* CORE_ENGINES_CAPTURE_ENGINE_H */
