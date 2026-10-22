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

#include <sys/timerfd.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <limits>

#include <utils/Errors.h>
#include <utils/Log.h>
#include <utils/Timers.h>

#include "3a/AutoFocusController.h"
#include "3a/RequestExposure.h"
#include "DeviceErrorReporter.h"
#include "IspEngine.h"
#include "IspPipeline.h"
#include "SensorWriter.h"
#include "StatsEngine.h"
#include "mochacamera/ports/ResultSink.h"
#include "sensor/SensorConfig.h"

#undef  LOG_TAG
#define LOG_TAG "Cam-CaptureEngine"

namespace mocha::camera {

using android::DelayedControls;

CaptureEngine::CaptureEngine(const Deps &d)
    : deps(d), permits(kPermits, &wakeFd), binder(d.controls, d.sensorCfg) {}

CaptureEngine::~CaptureEngine() {
    stop();
}

bool CaptureEngine::start() {
    if (engine)
        return false;
    binder.reset(deps.source->resolution().height);

    stallTimer.reset(::timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK | TFD_CLOEXEC));
    writeTimer.reset(::timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK | TFD_CLOEXEC));
    if (!stallTimer.valid() || !writeTimer.valid()) {
        ALOGE("timers: timerfd_create failed, errno=%d", errno);
        return false;
    }
    pending.reset();
    {
        /* The first frame runs on what configure wrote. */
        const DelayedControls::Batch e = deps.controls->inEffect(0);
        armStall(deps.sensorCfg->frameDurationNs(
                e.val[DelayedControls::FRAME_LENGTH],
                e.val[DelayedControls::LINE_LENGTH]));
    }

    /* Every request is taken as it comes: its frame is chosen then. No
     * stop grace: the only thing a job waits for is the sensor. */
    engine = std::make_unique<Eng>(
            "CamCapture",
            [this](Eng::Id, Job &job) { return advance(job); },
            EngineOptions{.maxRunning = std::numeric_limits<size_t>::max(),
                          .stopGrace  = std::chrono::milliseconds(0)});

    FrameSource *source = deps.source;
    engine->watch(source->fd(), [this, source](short revents) {
        onFrames(source->drain(revents));
    });
    engine->watch(source->returnedFd(), [source](short) {
        source->requeueReturned();
    });
    engine->watch(stallTimer.get(), [this](short) { onStall(); });
    engine->watch(writeTimer.get(), [this](short) {
        uint64_t expirations = 0;
        if (::read(writeTimer.get(), &expirations, sizeof(expirations)) ==
            (ssize_t)sizeof(expirations))
            writePending();
    });
    return engine->start();
}

void CaptureEngine::stop() {
    if (!engine)
        return;
    engine->stop();
    engine.reset();
    stallTimer.reset();
    writeTimer.reset();
    pending.reset();
}

void CaptureEngine::writePending() {
    if (!pending)
        return;
    if (pending->changed)
        deps.writer->write(pending->values);
    pending.reset();
}

void CaptureEngine::armStall(int64_t periodNs) {
    const int64_t ns = periodNs + kStallSlackNs;
    itimerspec it{};
    it.it_value.tv_sec  = (time_t)(ns / 1000000000LL);
    it.it_value.tv_nsec = (long)(ns % 1000000000LL);
    if (::timerfd_settime(stallTimer.get(), 0, &it, nullptr) != 0)
        ALOGE("stall timer: timerfd_settime failed, errno=%d", errno);
}

void CaptureEngine::onStall() {
    uint64_t expirations = 0;
    if (::read(stallTimer.get(), &expirations, sizeof(expirations)) !=
        (ssize_t)sizeof(expirations))
        return;
    ALOGE("no frame from the sensor for its period and %lld ms",
          (long long)(kStallSlackNs / 1000000));
    if (deps.deviceError)
        deps.deviceError->report("sensor stopped sending frames");
}

bool CaptureEngine::submit(FrameToken frame) {
    return engine && engine->submit(Job{std::move(frame), nullptr,
                                        std::nullopt, false}).has_value();
}

void CaptureEngine::onFrames(std::vector<BayerLease> frames) {
    for (BayerLease &lease : frames) {
        android::FrameBinder::Arrival a = binder.arrive(std::move(lease));
        const uint32_t seq = a.sensorFrame;
        if (!a.settled.empty())
            wakeFd.notify();

        /* The VI's wait now, in the blanking: it starts waiting on the
         * next frame's successor as the next frame starts, and the
         * write inside that frame may stretch it. */
        deps.writer->prepare(a.values);

        /* A write still pending is a frame late -- this thread fell a
         * frame behind; it goes now, and its frame's record may be off
         * by the frame. */
        if (pending) {
            ALOGW("sensor frame %u: write a frame late", seq - 1);
            writePending();
        }
        const int64_t at = a.nextFrameStartNs + kWriteAfterStartNs;
        a.settled.clear();
        pending = std::move(a);
        if (at <= systemTime()) {
            writePending();
        } else {
            itimerspec it{};
            it.it_value.tv_sec  = (time_t)(at / 1000000000LL);
            it.it_value.tv_nsec = (long)(at % 1000000000LL);
            if (::timerfd_settime(writeTimer.get(), TFD_TIMER_ABSTIME, &it,
                                  nullptr) != 0) {
                ALOGE("write timer: timerfd_settime failed, errno=%d", errno);
                writePending();
            }
        }

        const DelayedControls::Batch next = deps.controls->inEffect(seq + 1);
        armStall(deps.sensorCfg->frameDurationNs(
                next.val[DelayedControls::FRAME_LENGTH],
                next.val[DelayedControls::LINE_LENGTH]));
    }
}

Engine<CaptureEngine::Job, std::monostate>::Step CaptureEngine::advance(Job &job) {
    /* Drained before anything is looked at: a frame, a permit or a
     * wake() arriving after this point wakes the next poll. */
    wakeFd.drain();
    Frame &f = *job.frame;
    const auto again = [this, &job] { return advance(job); };

    if (!f.status.ok()) {
        if (job.ticket)
            binder.cancel(*job.ticket);
        return Eng::Step::done();
    }

    if (!job.ticket) {
        job.ticket = binder.request(
                android::requestExposure(f.settings, *deps.sensorCfg,
                                         deps.exposureTarget, deps.evStep),
                f.settings.stillCapture);
        if (deps.af)
            deps.af->onSettings(f.settings, f.frameNumber);
    }
    android::FrameBinder::Ticket &t = *job.ticket;
    if (t.lost) {
        /* Its frame was missed, and no frame ran on its settings. */
        f.status.fail(android::NOT_ENOUGH_DATA);
        return Eng::Step::done();
    }
    if (!t.bayer)
        return Eng::Step::wait(wakeFd.fd(), again);

    if (!job.shutterSent) {
        job.shutterSent   = true;
        f.sensorFrame     = t.sensorFrame;
        f.exposureStartNs = t.exposureStartNs;
        f.inEffect        = t.inEffect;
        f.tShutter        = systemTime();
        f.hasBayer.store(true, std::memory_order_release);
        deps.sink->shutter(f.frameNumber, f.exposureStartNs);
    }

    if (!job.permit) {
        job.permit = permits.tryAcquire();
        if (!job.permit)
            return Eng::Step::wait(wakeFd.fd(), again);
    }

    f.tBayerDq = systemTime();
    f.bayer    = std::move(*t.bayer);
    t.bayer.reset();
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
    if (!deps.stats || !deps.inputRing) return;

    /* Every frame: the stats engine paces itself, a phase per job. */
    const int slot = f.bayer.frame().slot;
    deps.inputRing->invalidateBayer(slot);
    const void *data = deps.inputRing->bayerHost(slot);
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
    job.sequence        = f.sensorFrame;
    job.exposureStartNs = f.exposureStartNs;
    job.inEffect        = f.inEffect;

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
