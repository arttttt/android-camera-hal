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

#include "Engine.h"
#include "EventFd.h"
#include "Frame.h"
#include "PermitPool.h"
#include "mochacamera/ports/FrameSource.h"

namespace android {
class AutoFocusController;
class DeviceErrorReporter;
class IspInputRing;
class Pipeline;
}

namespace mocha::camera {

class IspEngine;
class StatsEngine;

/* Where a frame meets the sensor: one thread for the request's settings,
 * the shutter, and the Bayer, in that order, frame after frame.
 *
 * The settings and the shutter go in step with the capture, not ahead
 * of it: ApplySettings writes the sensor and takes the IPA's decision
 * filed for this very frame number, and the shutter is stamped as it
 * runs, so both must run as the frame is captured -- an engine of their
 * own would race through the whole queue at once.
 *
 * A frame's job, one at a time (maxRunning 1 -- a second waiting job
 * would take a good frame from the first):
 *   1. a permit for the demosaic engine; without one, wait. This is
 *      what bounds the capture slots held between here and the GPU, and
 *      why the frame the GPU gets is the freshest one;
 *   2. ApplySettings, ShutterNotify;
 *   3. the newest Bayer; without one, wait;
 *   4. the crop, a lease to the stats engine, the frame and its permit
 *      to the demosaic engine.
 * A frame cancelled at any point -- flush, the AE mode switch -- ends
 * its job at the next look; wake() makes the look happen now, which is
 * how a long manual exposure is cut short.
 *
 * The thread also owns the V4L2 queue: the node is watched with the
 * job, drained to its newest frame as frames land, and slots that
 * leases give back are queued here, so V4L2 has one writer. */
class CaptureEngine {
public:
    struct Deps {
        FrameSource                  *source;
        android::IspInputRing        *inputRing;   /* CPU view of a slot, for stats */
        android::AutoFocusController *af;          /* may be null: focus ROI */
        android::Pipeline            *settings;    /* ApplySettings, ShutterNotify */
        IspEngine                    *demosaic;
        StatsEngine                  *stats;       /* may be null */
        android::DeviceErrorReporter *deviceError; /* may be null */
    };

    explicit CaptureEngine(const Deps &deps);
    ~CaptureEngine();

    CaptureEngine(const CaptureEngine &) = delete;
    CaptureEngine &operator=(const CaptureEngine &) = delete;

    bool start();
    void stop();

    /* False once stopped; the token goes, and the frame with it to its
     * result. */
    bool submit(FrameToken frame);

    /* Have the waiting frame look at its status again. */
    void wake() { wakeFd.notify(); }

private:
    struct Job {
        FrameToken            frame;
        std::optional<Permit> permit;
        bool                  settingsApplied = false;
    };
    using Eng = Engine<Job, std::monostate>;

    Eng::Step advance(Job &job);
    void crop(Frame &frame);
    void dispatchStats(Frame &frame);

    /* Frames for the demosaic engine at once: those on the GPU, and one
     * handed over and waiting -- the three-of-eight slot bound the
     * blocking queues used to give (with the newest frame the source
     * keeps). */
    static constexpr size_t kPermits = 2;

    Deps deps;
    /* Woken by a new frame, a permit coming back, and wake(). */
    android::EventFd wakeFd;
    PermitPool permits;
    std::unique_ptr<Eng> engine;
};

} /* namespace mocha::camera */

#endif /* CORE_ENGINES_CAPTURE_ENGINE_H */
