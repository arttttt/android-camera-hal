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

#include "IspEngine.h"

#include <chrono>

#include <utils/Log.h>

#include "DeviceErrorReporter.h"
#include "FinishEngine.h"
#include "stages/DemosaicBlitStage.h"

#undef  LOG_TAG
#define LOG_TAG "Cam-IspEngine"

namespace mocha::camera {

namespace {
/* Longest a frame's GPU work may take before the GPU counts as hung;
 * VulkanIspPipeline's own slot-fence limit is the same. Also how long a
 * stop waits for the fence of a frame on the GPU. */
constexpr std::chrono::milliseconds kGpuFenceTimeout{1000};
} /* namespace */

IspEngine::IspEngine(const Deps &d) : deps(d) {}

IspEngine::~IspEngine() {
    stop();
}

bool IspEngine::start() {
    if (engine)
        return false;
    engine = std::make_unique<Eng>(
            "CamIsp",
            [this](Eng::Id, Job &job) { return handle(job); },
            EngineOptions{.maxRunning = kDepth, .stopGrace = kGpuFenceTimeout});
    android::DeviceErrorReporter *deviceError = deps.deviceError;
    engine->setStallWatch(kGpuFenceTimeout, [deviceError] {
        ALOGE("GPU fence not signalled after %lld ms",
              (long long)kGpuFenceTimeout.count());
        if (deviceError)
            deviceError->report("ISP GPU fence never signalled");
    });
    return engine->start();
}

void IspEngine::stop() {
    if (!engine)
        return;
    engine->stop();
    engine.reset();
}

bool IspEngine::submit(FrameToken frame, Permit permit) {
    return engine &&
           engine->submit(Job{std::move(frame), std::move(permit), android::UniqueFd()})
                   .has_value();
}

Engine<IspEngine::Job, std::monostate>::Step IspEngine::handle(Job &job) {
    /* A frame cancelled or failed on its way here skips the GPU; its
     * token goes with the job, and the frame on to its result. */
    if (!job.frame->status.ok())
        return Eng::Step::done();

    job.fence = deps.demosaic->process(*job.frame);
    if (!job.fence.valid())
        return passOn(job);
    return Eng::Step::wait(job.fence.get(), [this, &job] { return passOn(job); });
}

Engine<IspEngine::Job, std::monostate>::Step IspEngine::passOn(Job &job) {
    job.fence.reset();
    /* The GPU has read the frame's Bayer; nothing after it does (the
     * stats hold a lease of their own). The slot goes back to the
     * sensor now, with the permit, so the capture slots held past
     * capture stay bounded by the permits however far the finish
     * engine falls behind. */
    job.frame->bayer.reset();
    /* A frame whose demosaic failed has no GPU work to finish; the
     * finish engine still lets its JPEG snapshots go. */
    if (!deps.finish->submit(std::move(job.frame)))
        ALOGE("the finish engine is stopped; frame given up");
    return Eng::Step::done();
}

} /* namespace mocha::camera */
