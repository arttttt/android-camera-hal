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

#ifndef CORE_ENGINES_ISP_ENGINE_H
#define CORE_ENGINES_ISP_ENGINE_H

#include <memory>
#include <variant>

#include "Engine.h"
#include "Frame.h"
#include "PermitPool.h"
#include "UniqueFd.h"

namespace android {
class DemosaicBlitStage;
class DeviceErrorReporter;
}

namespace mocha::camera {

class FinishEngine;

/* The GPU half of a frame: record the demosaic and every output's blit
 * into one submit, then wait for its fence without blocking anything
 * else on the thread, and pass the frame to the finish engine.
 *
 * One frame at a time (maxRunning 1, as PIPELINE_MAX_IN_FLIGHT was):
 * the GPU is the bottleneck — ~18-20 ms a frame in 1080p30 recording
 * on average (demosaic 13-14 ms, outputs 5-6 ms), 12-14 ms with the
 * clocks pinned — and a deeper ring only adds latency and lets
 * completions clump.
 *
 * A job carries the capture engine's permit; it goes back when the job
 * ends -- fence signalled, frame given up, or engine stopped -- and the
 * frame's Bayer lease goes back as the fence signals, which together
 * bound the capture slots held past capture.
 *
 * A fence not signalled within a second is a hung GPU: reported once
 * as a device error; the frame stays in flight until the framework
 * closes the camera, whose stop gives it up after the same second. */
class IspEngine {
public:
    struct Job {
        FrameToken        frame;
        Permit            permit;
        android::UniqueFd fence;
    };

    struct Deps {
        android::DemosaicBlitStage   *demosaic;
        FinishEngine                 *finish;
        android::DeviceErrorReporter *deviceError;   /* may be null */
    };

    /* Frames on the GPU at once. */
    static constexpr size_t kDepth = 1;

    explicit IspEngine(const Deps &deps);
    ~IspEngine();

    IspEngine(const IspEngine &) = delete;
    IspEngine &operator=(const IspEngine &) = delete;

    bool start();
    void stop();

    /* False once stopped; the job goes, its permit and its frame's token
     * with it. */
    bool submit(FrameToken frame, Permit permit);

private:
    using Eng = Engine<Job, std::monostate>;

    Eng::Step handle(Job &job);
    Eng::Step passOn(Job &job);

    Deps deps;
    std::unique_ptr<Eng> engine;
};

} /* namespace mocha::camera */

#endif /* CORE_ENGINES_ISP_ENGINE_H */
