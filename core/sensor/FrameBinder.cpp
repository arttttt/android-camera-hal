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
    hiddenAhead.clear();
    hiddenPassed = 0;
    lineLength.reset();
    unserved.clear();
    catchingUp = false;
    filedBy.clear();
}

uint32_t FrameBinder::frameOf(uint32_t sequence) const {
    uint32_t frame = sequence + hiddenPassed;
    for (const Hidden &h : hiddenAhead)
        if (sequence > h.afterSequence) ++frame;
    return frame;
}

bool FrameBinder::hidden(uint32_t frame) const {
    for (const Hidden &h : hiddenAhead)
        if (h.frame == frame) return true;
    return false;
}

std::shared_ptr<FrameBinder::Ticket> FrameBinder::request(
        const DelayedControls::Batch &settings, bool stillCapture) {
    /* The first frame its settings can still be filed for, and one past
     * the frame of the request before it. */
    const uint32_t first = controls->firstFileable();
    while (!unserved.empty() && *unserved.begin() < first)
        unserved.erase(unserved.begin());
    const auto free = [this](uint32_t f) {
        while (unserved.count(f)) ++f;
        return f;
    };
    uint32_t frame = first;
    if (anyRequest && frame <= lastAsked)
        frame = lastAsked + 1;
    frame = free(frame);

    /* A stream run dry, or a request sent right behind one given its
     * frame early, before a frame came: the next frame nobody asked
     * for, if it comes before the one the settings can be on. */
    const bool dry = !anyRequest || (anyFrame && lastFrame > lastAsked);
    uint32_t early = frame;
    if (!stillCapture && (dry || catchingUp)) {
        early = anyFrame ? lastFrame + 1 : 0;
        if (anyRequest && early <= lastAsked)
            early = lastAsked + 1;
        while (hidden(early) || unserved.count(early)) ++early;
    }
    catchingUp = early < frame;

    if (stillCapture) {
        DelayedControls::Batch after =
            streamSettings ? *streamSettings : controls->planned(frame + 1);
        /* A line length its own, written with the rest two frames
         * before it, hides the frame before it, and going back hides
         * the frame after it. No frame comes for those, and none for
         * the frames two on from them: their writes would go out in
         * the hidden frames, which have no start. None of them is
         * given to a request. */
        const bool hides = cfg->lineLengthHidesFrame &&
            settings.val[DelayedControls::LINE_LENGTH] !=
                after.val[DelayedControls::LINE_LENGTH];
        if (hides && anyRequest && frame - 1 == lastAsked) {
            frame = free(frame + 1);
            if (!streamSettings) after = controls->planned(frame + 1);
        }
        controls->file(frame, settings);
        controls->file(frame + 1, after);
        if (hides)
            unserved.insert({frame - 1, frame + 1, frame + 3});
    } else {
        controls->file(frame, settings);
        streamSettings = settings;
    }
    anyRequest = true;
    lastAsked  = catchingUp ? early : frame;

    auto ticket = std::make_shared<Ticket>();
    ticket->frame    = lastAsked;
    ticket->settings = settings;
    ticket->early    = catchingUp;
    ticket->filedFor = frame;
    filedBy[frame]   = ticket;
    wanted.push_back(ticket);
    return ticket;
}

void FrameBinder::cancel(const Ticket &t) {
    if (t.bayer || t.lost)
        return;
    const auto it = filedBy.find(t.filedFor);
    if (it == filedBy.end() || it->second.lock().get() != &t)
        return;
    controls->unfile(t.filedFor);
    filedBy.erase(it);
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
    const uint32_t frame = frameOf(sequence);
    bool hides = false;
    if (anyStart && frame <= lastStart) {
        ALOGW("frame %u started after frame %u started: not written on",
              frame, lastStart);
    } else if (anyFrame && frame <= lastFrame) {
        ALOGW("frame %u started after frame %u came: not written on",
              frame, lastFrame);
    } else {
        uint32_t next = lastStart + 1;
        while (anyStart && hidden(next)) ++next;
        if (anyStart && frame != next) {
            /* Starts the driver did not report, or the capture passed
             * over: no write was made in those frames, and
             * DelayedControls lets this tick overtake them. */
            startsMissed += frame - next;
            ALOGW("frame %u started after %u: %u frame start(s) not "
                  "written on", frame, lastStart, frame - next);
        }
        anyStart  = true;
        lastStart = frame;
        s.ticked  = true;
        s.values  = controls->tick(frame, &s.changed);
        const int32_t ll = s.values.val[DelayedControls::LINE_LENGTH];
        hides = cfg->lineLengthHidesFrame && lineLength && *lineLength != ll;
        if (hides)
            hiddenAhead.push_back({frame + 1, sequence});
        lineLength = ll;
    }

    /* From this start to the next: this frame, and the one its write
     * hides; and the same from the next start, on what the next tick
     * would write (a line length new to it hides one more). */
    const auto period = [this](const DelayedControls::Batch &b) {
        return cfg->frameDurationNs(b.val[DelayedControls::FRAME_LENGTH],
                                    b.val[DelayedControls::LINE_LENGTH]);
    };
    s.periodNs = period(controls->inEffect(frame));
    int64_t untilNext = s.periodNs;
    if (hides)
        untilNext += cfg->frameSpanNs(controls->inEffect(frame + 1));
    const uint32_t nextTick = frame + (hides ? 2 : 1);
    const DelayedControls::Batch now = s.ticked ? s.values : controls->peek(frame);
    const DelayedControls::Batch next = controls->peek(nextTick);
    int64_t fromNext = period(next);
    if (cfg->lineLengthHidesFrame &&
        next.val[DelayedControls::LINE_LENGTH] != now.val[DelayedControls::LINE_LENGTH]) {
        DelayedControls::Batch held = controls->inEffect(nextTick + 1);
        held.val[DelayedControls::FRAME_LENGTH] =
            std::max(held.val[DelayedControls::FRAME_LENGTH],
                     next.val[DelayedControls::FRAME_LENGTH]);
        held.val[DelayedControls::LINE_LENGTH] = next.val[DelayedControls::LINE_LENGTH];
        held.val[DelayedControls::EXPOSURE] =
            std::max(held.val[DelayedControls::EXPOSURE],
                     next.val[DelayedControls::EXPOSURE]);
        fromNext += cfg->frameSpanNs(held);
    }
    s.waitPeriodNs = std::max({untilNext, s.ticked ? period(s.values) : 0,
                               fromNext});
    return s;
}

FrameBinder::Arrival FrameBinder::arrive(mocha::camera::BayerLease lease) {
    const uint32_t seq   = frameOf(lease.frame().sequence);
    const int64_t  endNs = lease.frame().timestampNs;

    Arrival a;
    a.sensorFrame = seq;
    if (anyFrame && seq <= lastFrame) {
        /* A number again, or back: not a frame to give anyone. */
        ALOGW("frame %u came after frame %u: given back", seq, lastFrame);
        return a;
    }
    uint32_t next = lastFrame + 1;
    while (anyFrame && hidden(next)) ++next;
    if (anyFrame && seq != next) {
        /* Frames the VI had no buffer for. */
        framesMissed += seq - next;
        ALOGW("frame %u came after %u: %u frame(s) not captured",
              seq, lastFrame, seq - next);
    }
    anyFrame  = true;
    lastFrame = seq;
    /* The requests sent from here on are not the restart's burst. */
    catchingUp = false;
    /* Hidden frames both the starts and the frames are past. */
    while (!hiddenAhead.empty() && hiddenAhead.front().frame < lastFrame &&
           hiddenAhead.front().frame < lastStart) {
        hiddenAhead.pop_front();
        ++hiddenPassed;
    }

    /* The request that asked for this frame, and any whose frame the
     * capture missed: the frame serves each that it ran on the settings
     * of, every control on the frame its delay put it on. One that it
     * did not is lost -- a missed frame, or the write a missed start's
     * tick did not make. A frame nobody takes goes back with its
     * lease. */
    const DelayedControls::Batch ran = controls->inEffectFor(seq);
    for (std::shared_ptr<Ticket> t = front(); t && t->frame <= seq; t = front()) {
        wanted.pop_front();
        /* One given its frame early takes it as it ran. */
        bool own = true;
        for (int id = 0; id < DelayedControls::COUNT && !t->early; ++id)
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
    filedBy.erase(filedBy.begin(), filedBy.lower_bound(seq));
    return a;
}

} /* namespace android */
