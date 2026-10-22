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

#include <utils/Log.h>

#include "sensor/SensorConfig.h"

namespace android {

FrameBinder::FrameBinder(DelayedControls *c, const SensorConfig *s)
    : controls(c), cfg(s) {}

void FrameBinder::reset(uint32_t l) {
    lines = l;
    sequence.reset();
    wanted.clear();
    anyRequest = false;
    lastFrame  = 0;
}

std::shared_ptr<FrameBinder::Ticket> FrameBinder::request(
        const DelayedControls::Batch &settings) {
    /* The first frame its settings can still be filed for, and one past
     * the frame of the request before it. */
    uint32_t frame = controls->firstFileable();
    if (anyRequest && frame <= lastFrame)
        frame = lastFrame + 1;
    anyRequest = true;
    lastFrame  = frame;
    controls->file(frame, settings);

    auto ticket = std::make_shared<Ticket>();
    ticket->frame    = frame;
    ticket->settings = settings;
    wanted.push_back(ticket);
    return ticket;
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

FrameBinder::Arrival FrameBinder::arrive(mocha::camera::BayerLease lease) {
    const uint32_t driverSeq = lease.frame().sequence;
    const int64_t  endNs     = lease.frame().timestampNs;

    Arrival a;
    a.sensorFrame = sequence.next(driverSeq, endNs, [this](uint32_t seq) {
        const DelayedControls::Batch e = controls->inEffect(seq);
        return cfg->frameDurationNs(e.val[DelayedControls::FRAME_LENGTH],
                                    e.val[DelayedControls::LINE_LENGTH]);
    });
    a.values = controls->tick(a.sensorFrame, &a.changed);
    a.nextFrameStartNs = cfg->nextFrameStartNs(
            endNs, controls->inEffect(a.sensorFrame), lines);

    /* The request that asked for this frame, and any whose frame the
     * capture missed: the frame serves each that it ran on the settings
     * of. One that it did not is lost -- a missed frame, or the write a
     * missed frame's tick did not make. A frame nobody takes goes back
     * with its lease. */
    const DelayedControls::Batch here = controls->inEffect(a.sensorFrame);
    for (std::shared_ptr<Ticket> t = front();
         t && t->frame <= a.sensorFrame; t = front()) {
        wanted.pop_front();
        bool own = true;
        for (int id = 0; id < DelayedControls::COUNT; ++id)
            if (t->settings.has[id] && t->settings.val[id] != here.val[id])
                own = false;
        if (own) {
            give(*t, lease, a.sensorFrame, endNs);
        } else {
            t->lost = true;
            ALOGW("sensor frame %u did not run on its request's settings "
                  "(frame %u came); the request is lost",
                  t->frame, a.sensorFrame);
        }
        a.settled.push_back(std::move(t));
    }
    return a;
}

} /* namespace android */
