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

#include "CaptureEngine.h"

#include <chrono>

#include <utils/Errors.h>
#include <utils/Log.h>
#include <utils/Timers.h>

#include "3a/AutoFocusController.h"
#include "DeviceErrorReporter.h"
#include "IspEngine.h"
#include "IspPipeline.h"
#include "Pipeline.h"
#include "StatsEngine.h"

#undef  LOG_TAG
#define LOG_TAG "Cam-CaptureEngine"

namespace mocha::camera {

namespace {
/* No frame for this long while one is wanted: the sensor or the
 * capture hardware has stopped (a manual exposure is at most 1 s, and
 * the driver stretches a frame to ~0.85 s at most). Waiting for a
 * permit counts too -- a GPU stuck that long has been reported by the
 * demosaic engine already, and the report goes out once. */
constexpr std::chrono::milliseconds kStall{3000};
} /* namespace */

CaptureEngine::CaptureEngine(const Deps &d)
    : deps(d), permits(kPermits, &wakeFd) {}

CaptureEngine::~CaptureEngine() {
    stop();
}

bool CaptureEngine::start() {
    if (engine)
        return false;
    /* No stop grace: the only thing a job waits for is the sensor. */
    engine = std::make_unique<Eng>(
            "CamCapture",
            [this](Eng::Id, Job &job) { return advance(job); },
            EngineOptions{.maxRunning = 1, .stopGrace = std::chrono::milliseconds(0)});

    FrameSource *source = deps.source;
    android::EventFd *wake = &wakeFd;
    engine->watch(source->fd(), [source, wake](short revents) {
        if (source->drainToLatest(revents))
            wake->notify();
    });
    engine->watch(source->returnedFd(), [source](short) {
        source->requeueReturned();
    });
    android::DeviceErrorReporter *deviceError = deps.deviceError;
    engine->setStallWatch(kStall, [deviceError] {
        ALOGE("no frame to capture for %lld ms", (long long)kStall.count());
        if (deviceError)
            deviceError->report("sensor stopped sending frames");
    });
    return engine->start();
}

void CaptureEngine::stop() {
    if (!engine)
        return;
    engine->stop();
    engine.reset();
}

bool CaptureEngine::submit(FrameToken frame) {
    return engine && engine->submit(Job{std::move(frame), std::nullopt}).has_value();
}

Engine<CaptureEngine::Job, std::monostate>::Step CaptureEngine::advance(Job &job) {
    /* Drained before anything is looked at: a frame, a permit or a
     * wake() arriving after this point wakes the next poll. */
    wakeFd.drain();
    Frame &f = *job.frame;
    const auto again = [this, &job] { return advance(job); };

    if (!f.status.ok())
        return Eng::Step::done();

    if (!job.permit) {
        job.permit = permits.tryAcquire();
        if (!job.permit)
            return Eng::Step::wait(wakeFd.fd(), again);
    }

    if (!job.settingsApplied) {
        job.settingsApplied = true;
        deps.settings->run(f);
        if (!f.status.ok())
            return Eng::Step::done();
    }

    std::optional<BayerLease> lease = deps.source->takeLatest();
    if (!lease)
        return Eng::Step::wait(wakeFd.fd(), again);

    f.tBayerDq = systemTime();
    f.bayer    = std::move(*lease);
    f.hasBayer.store(true, std::memory_order_release);
    crop(f);
    dispatchStats(f);

    const uint32_t frameNumber = f.frameNumber;
    if (!deps.demosaic->submit(std::move(job.frame), std::move(*job.permit)))
        ALOGE("frame %u: the demosaic engine is stopped", frameNumber);
    return Eng::Step::done();
}

void CaptureEngine::crop(Frame &f) {
    const Size res    = deps.source->resolution();
    const Size sensor = deps.source->sensorResolution();

    int cropX = 0;
    int cropY = 0;
    int cropW = res.width;
    int cropH = res.height;

    if (f.settings.crop) {
        const CropRegion &c = *f.settings.crop;
        cropX = c.left   * (int)res.width  / (int)sensor.width;
        cropY = c.top    * (int)res.height / (int)sensor.height;
        cropW = c.width  * (int)res.width  / (int)sensor.width;
        cropH = c.height * (int)res.height / (int)sensor.height;
        if (cropX < 0) cropX = 0;
        if (cropY < 0) cropY = 0;
        if (cropW < 16) cropW = 16;
        if (cropH < 16) cropH = 16;
        if (cropX + cropW > (int)res.width)  cropW = (int)res.width  - cropX;
        if (cropY + cropH > (int)res.height) cropH = (int)res.height - cropY;
    }

    f.cropX = cropX;
    f.cropY = cropY;
    f.cropW = cropW;
    f.cropH = cropH;
}

void CaptureEngine::dispatchStats(Frame &f) {
    if (!deps.stats || !deps.isp) return;

    /* Every frame: the stats engine paces itself, a phase per job. */
    const int slot = f.bayer.frame().slot;
    deps.isp->invalidateBayer(slot);
    const void *data = deps.isp->bayerHost(slot);
    if (!data) return;

    const Size res = deps.source->resolution();

    StatsEngine::Job job;
    /* The stats read the frame past the frame's own demosaic (the
     * cycle's second phase runs on the next frame's job): a lease of
     * their own. */
    job.bayer           = f.bayer.share();
    job.data            = data;
    job.width           = res.width;
    job.height          = res.height;
    job.pixFmt          = f.bayer.frame().pixFmt;
    job.sequence        = f.frameNumber;
    job.exposureStartNs = f.bayer.frame().exposureStartNs();

    /* The AF region of the request; without AF, the
     * compile-time centre rectangle. */
    if (deps.af) {
        const android::AutoFocusController::FocusRoi roi = deps.af->currentFocusRoi();
        job.focusRoi.pyLo = roi.pyLo;
        job.focusRoi.pyHi = roi.pyHi;
        job.focusRoi.pxLo = roi.pxLo;
        job.focusRoi.pxHi = roi.pxHi;
    } else {
        job.focusRoi.pyLo = android::IpaStats::FOCUS_ROI_PY_LO;
        job.focusRoi.pyHi = android::IpaStats::FOCUS_ROI_PY_HI;
        job.focusRoi.pxLo = android::IpaStats::FOCUS_ROI_PX_LO;
        job.focusRoi.pxHi = android::IpaStats::FOCUS_ROI_PX_HI;
    }

    deps.stats->submit(std::move(job));
}

} /* namespace mocha::camera */
