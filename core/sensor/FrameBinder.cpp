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

#undef  LOG_TAG
#define LOG_TAG "Cam-FrameBinder"

#include "FrameBinder.h"

#include <algorithm>

#include <utils/Log.h>

#include "sensor/SensorConfig.h"

namespace android {

FrameBinder::FrameBinder(DelayedControls *c, const SensorConfig *s)
    : controls(c), cfg(s) {}

void FrameBinder::reset(uint32_t l) {
    lines = l;
    wanted.clear();
    anyRequest   = false;
    lastAsked    = 0;
    streamSettings.reset();
    anyFrame     = false;
    lastFrame    = 0;
    anyStart     = false;
    lastStart    = 0;
    framesMissed = 0;
    startsMissed = 0;
}

std::shared_ptr<FrameBinder::Ticket> FrameBinder::request(
        const DelayedControls::Batch &settings, bool stillCapture) {
    /* The first frame its settings can still be filed for, and one past
     * the frame of the request before it. */
    uint32_t frame = controls->firstFileable();
    if (anyRequest && frame <= lastAsked)
        frame = lastAsked + 1;
    anyRequest = true;
    lastAsked  = frame;
    if (stillCapture) {
        const DelayedControls::Batch after =
            streamSettings ? *streamSettings : controls->planned(frame + 1);
        controls->file(frame, settings);
        controls->file(frame + 1, after);
    } else {
        controls->file(frame, settings);
        streamSettings = settings;
    }

    auto ticket = std::make_shared<Ticket>();
    ticket->frame    = frame;
    ticket->settings = settings;
    wanted.push_back(ticket);
    return ticket;
}

void FrameBinder::cancel(const Ticket &t) {
    if (t.bayer || t.lost)
        return;
    controls->unfile(t.frame);
}

std::shared_ptr<FrameBinder::Ticket> FrameBinder::front() {
    while (!wanted.empty()) {
        if (std::shared_ptr<Ticket> t = wanted.front().lock())
            return t;
        wanted.pop_front();
    }
    return nullptr;
}

void FrameBinder::give(Ticket &t, const mocha::camera::BayerLease &lease,
                       uint32_t sensorFrame, int64_t endNs) {
    t.sensorFrame     = sensorFrame;
    t.inEffect        = controls->inEffect(sensorFrame);
    t.exposureStartNs = cfg->exposureStartNs(endNs, t.inEffect, lines);
    t.bayer           = lease.share();
}

FrameBinder::Start FrameBinder::start(uint32_t sequence) {
    Start s;
    if (!anyStart && sequence != 0) {
        /* The stream's first start is not 0 (a frame came before the
         * capture's first): what was asked for moves on by it. */
        controls->rebase(sequence);
        for (const std::weak_ptr<Ticket> &w : wanted)
            if (std::shared_ptr<Ticket> t = w.lock()) t->frame += sequence;
        if (anyRequest) lastAsked += sequence;
    }
    if (anyStart && sequence <= lastStart) {
        ALOGW("frame %u started after frame %u started: not written on",
              sequence, lastStart);
    } else if (anyFrame && sequence <= lastFrame) {
        ALOGW("frame %u started after frame %u came: not written on",
              sequence, lastFrame);
    } else {
        if (anyStart && sequence != lastStart + 1) {
            /* Starts the driver did not report, or the capture passed
             * over: no write was made in those frames, and
             * DelayedControls lets this tick overtake them. */
            startsMissed += sequence - lastStart - 1;
            ALOGW("frame %u started after %u: %u frame start(s) not "
                  "written on", sequence, lastStart, sequence - lastStart - 1);
        }
        anyStart  = true;
        lastStart = sequence;
        s.ticked  = true;
        s.values  = controls->tick(sequence, &s.changed);
    }
    const auto period = [this](const DelayedControls::Batch &b) {
        return cfg->frameDurationNs(b.val[DelayedControls::FRAME_LENGTH],
                                    b.val[DelayedControls::LINE_LENGTH]);
    };
    s.periodNs = period(controls->inEffect(sequence));
    s.waitPeriodNs = std::max(s.ticked ? period(s.values) : s.periodNs,
                              period(controls->peek(sequence + 1)));
    return s;
}

FrameBinder::Arrival FrameBinder::arrive(mocha::camera::BayerLease lease) {
    const uint32_t seq   = lease.frame().sequence;
    const int64_t  endNs = lease.frame().timestampNs;

    Arrival a;
    a.sensorFrame = seq;
    if (anyFrame && seq <= lastFrame) {
        /* A number again, or back: not a frame to give anyone. */
        ALOGW("frame %u came after frame %u: given back", seq, lastFrame);
        return a;
    }
    if (anyFrame && seq != lastFrame + 1) {
        /* Frames the VI had no buffer for. */
        framesMissed += seq - lastFrame - 1;
        ALOGW("frame %u came after %u: %u frame(s) not captured",
              seq, lastFrame, seq - lastFrame - 1);
    }
    anyFrame  = true;
    lastFrame = seq;

    /* The request that asked for this frame, and any whose frame the
     * capture missed: the frame serves each that it ran on the settings
     * of, every control on the frame its delay put it on. One that it
     * did not is lost -- a missed frame, or the write a missed start's
     * tick did not make. A frame nobody takes goes back with its
     * lease. */
    const DelayedControls::Batch ran = controls->inEffectFor(seq);
    for (std::shared_ptr<Ticket> t = front(); t && t->frame <= seq; t = front()) {
        wanted.pop_front();
        bool own = true;
        for (int id = 0; id < DelayedControls::COUNT; ++id)
            if (t->settings.has[id] && t->settings.val[id] != ran.val[id])
                own = false;
        if (own) {
            give(*t, lease, seq, endNs);
        } else {
            t->lost = true;
            ALOGW("sensor frame %u did not run on its request's settings "
                  "(frame %u came); the request is lost", t->frame, seq);
        }
        a.settled.push_back(std::move(t));
    }

    /* No request on an earlier frame is asked about any more. */
    controls->forgetBefore(seq);
    return a;
}

} /* namespace android */
