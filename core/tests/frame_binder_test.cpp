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

/* Host test for FrameBinder against a simulated sensor: a write made as
 * frame k arrives is in effect from frame k + delay, every frame is as
 * long as its own frame and line length, and the capture may miss a
 * frame. Each request must get the frame its settings were in effect
 * on, whatever the number of requests in flight. Run by
 * core/tests/run_host_tests.sh. */

#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <vector>

#include "mochacamera/ports/FrameSource.h"
#include "sensor/DelayedControls.h"
#include "sensor/FrameBinder.h"
#include "sensor/SensorConfig.h"
#include "Check.h"

using android::DelayedControls;
using android::FrameBinder;
using android::SensorConfig;
using mocha::camera::BayerFrame;
using mocha::camera::BayerLease;
using mocha::camera::SlotReturn;

using mocha::camera::test::check;
using mocha::camera::test::finish;

namespace {

constexpr int      kDelay = 3;
constexpr uint32_t kLines = 1080;

SensorConfig imx179() {
    SensorConfig c = SensorConfig::imx179();
    c.frameLenDefault   = 2564;
    c.frameLenMax       = 0xFFFF;
    c.lineTimeNs        = 13271;   /* HTS 3440 at 259.2 MHz */
    c.lineLengthDefault = 3440;
    c.lineLengthMax     = 0x7FFF;
    c.gainMin           = 256;
    c.gainDefault       = 256;
    c.gainMax           = 4096;
    c.exposureMin       = 13;
    c.exposureDefault   = 16000;
    c.exposureMax       = 99000;
    return c;
}

/* Slots the binder let go of. */
struct Slots : SlotReturn {
    int returned = 0;
    void returnSlot(int, uint32_t) override { ++returned; }
};

/* A request's settings: its own exposure, the default frame. */
DelayedControls::Batch settings(const SensorConfig &cfg, int32_t exposureUs) {
    DelayedControls::Batch b = cfg.configureDefaults();
    b.val[DelayedControls::EXPOSURE] = exposureUs;
    return b;
}

bool same(const DelayedControls::Batch &a, const DelayedControls::Batch &b) {
    for (int id = 0; id < DelayedControls::COUNT; ++id)
        if (a.val[id] != b.val[id]) return false;
    return true;
}

/* The sensor, the capture and the binder. frame() runs one sensor
 * frame: it runs on the latest write in effect by then, lasts what its
 * frame and line length make it, and -- unless the capture misses it --
 * arrives at the binder, whose write is in effect kDelay frames on. */
class Rig {
public:
    SensorConfig    cfg = imx179();
    DelayedControls dc{[] {
        DelayedControls::Config c;
        for (int &d : c.delay) d = kDelay;
        return c;
    }()};
    FrameBinder     binder{&dc, &cfg};
    Slots           slots;

    /* What each sensor frame really ran on. */
    std::vector<DelayedControls::Batch> ran;

    Rig() {
        dc.reset(cfg.configureDefaults());
        binder.reset(kLines);
        running = cfg.configureDefaults();
    }

    std::optional<FrameBinder::Arrival> frame(bool captured = true) {
        const uint32_t k = static_cast<uint32_t>(ran.size());
        auto w = writes.find(k);
        if (w != writes.end()) running = w->second;
        ran.push_back(running);
        endNs += cfg.frameDurationNs(running.val[DelayedControls::FRAME_LENGTH],
                                     running.val[DelayedControls::LINE_LENGTH]);
        if (!captured)
            return std::nullopt;

        BayerFrame f;
        f.slot        = static_cast<int>(k % 8);
        f.sequence    = driverSeq++;
        f.timestampNs = endNs;
        FrameBinder::Arrival a = binder.arrive(BayerLease(f, 0, &slots));
        if (a.changed)
            writes[k + kDelay] = a.values;
        return a;
    }

    int64_t frameEndNs() const { return endNs; }

private:
    std::map<uint32_t, DelayedControls::Batch> writes;
    DelayedControls::Batch running;
    int64_t  endNs = 1000000000;
    uint32_t driverSeq = 100;
};

struct InFlight {
    std::shared_ptr<FrameBinder::Ticket> ticket;
    DelayedControls::Batch               wanted;
};

/* N requests in flight, a new one sent as each one's frame comes -- the
 * framework's pace. Every request, in order, gets a frame that ran on
 * its own settings, the frame it asked for; with fewer requests in
 * flight than the delay covers, frames between them go unused and
 * nothing else changes. */
void testInFlight(size_t n) {
    Rig r;
    std::deque<InFlight> inFlight;
    int next = 0;
    const auto send = [&] {
        const DelayedControls::Batch s = settings(r.cfg, 1000 + 10 * next++);
        inFlight.push_back({r.binder.request(s, false), s});
    };
    for (size_t i = 0; i < n; ++i) send();

    const uint32_t kFrames = 400;
    int done = 0;
    bool ordered = true, own = true, asked = true;
    uint32_t lastFrame = 0;
    for (uint32_t k = 0; k < kFrames; ++k) {
        const std::optional<FrameBinder::Arrival> a = r.frame();
        if (!a || a->settled.empty()) continue;
        check(a->settled.size() == 1);
        const std::shared_ptr<FrameBinder::Ticket> &t = a->settled.front();
        const InFlight &front = inFlight.front();
        ordered = ordered && t == front.ticket
                          && (done == 0 || t->sensorFrame > lastFrame);
        own     = own && same(t->inEffect, front.wanted)
                      && same(r.ran[t->sensorFrame], front.wanted);
        asked   = asked && !t->lost && t->sensorFrame == t->frame;
        lastFrame = t->sensorFrame;
        inFlight.pop_front();
        ++done;
        send();
    }
    check(ordered);
    check(own);
    check(asked);
    check(r.binder.missed() == 0);

    /* A request sent as a frame arrives can get the frame kDelay + 1
     * on; n in flight fill that many of each kDelay + 1 frames, up to
     * all of them. */
    const size_t perCycle = n < (size_t)kDelay + 1 ? n : (size_t)kDelay + 1;
    const int expected = (int)(kFrames * perCycle / (kDelay + 1));
    check(done >= expected - (int)(2 * (kDelay + 1)));
    check(done <= expected + 1);
}

/* The write goes once the next frame has started: past the frame's end
 * by its own blanking -- the lines of its frame past those read out --
 * at its own line time, so a long frame's write waits its long blanking
 * and every mode's lands inside the next frame. */
void testNextFrameStart() {
    Rig r;
    const std::optional<FrameBinder::Arrival> a = r.frame();
    const int64_t blanking =
        (int64_t)(r.cfg.frameLenDefault - (int32_t)kLines) * r.cfg.lineTimeNs;
    check(a->nextFrameStartNs == r.frameEndNs() + blanking);

    DelayedControls::Batch b = r.cfg.configureDefaults();
    b.val[DelayedControls::FRAME_LENGTH] = 40000;
    b.val[DelayedControls::LINE_LENGTH]  = 2 * r.cfg.lineLengthDefault;
    check(r.cfg.nextFrameStartNs(1000, b, kLines)
          == 1000 + (40000 - (int64_t)kLines) * 2 * r.cfg.lineTimeNs);
}

/* A long manual exposure among short ones: its request gets the long
 * frame, the frames after it are numbered right (a 1.2 s frame is one
 * frame), and the requests after it get their own. */
void testLongFrame() {
    Rig r;
    const SensorConfig::FrameTiming t = r.cfg.timingForExposure(1200000);
    DelayedControls::Batch longShot = settings(r.cfg, 1200000);
    longShot.val[DelayedControls::FRAME_LENGTH] = t.frameLen;
    longShot.val[DelayedControls::LINE_LENGTH]  = t.lineLength;

    std::deque<InFlight> inFlight;
    for (int i = 0; i < 4; ++i) {
        const DelayedControls::Batch s = i == 1 ? longShot : settings(r.cfg, 2000 + i);
        inFlight.push_back({r.binder.request(s, i == 1), s});
    }

    bool own = true;
    int done = 0;
    for (uint32_t k = 0; k < 20 && !inFlight.empty(); ++k) {
        const std::optional<FrameBinder::Arrival> a = r.frame();
        if (!a || a->settled.empty()) continue;
        const std::shared_ptr<FrameBinder::Ticket> &got = a->settled.front();
        own = own && a->settled.size() == 1 && got == inFlight.front().ticket
                  && same(r.ran[got->sensorFrame], inFlight.front().wanted)
                  && same(got->inEffect, inFlight.front().wanted);
        if (done == 1) {
            const int64_t period = r.cfg.frameDurationNs(t.frameLen, t.lineLength);
            check(period > 1190000000 && period < 1260000000);
            /* Its exposure began 1.2 s and a read-out before its end. */
            const int64_t readout = (int64_t)kLines * r.cfg.lineTimeNsAt(t.lineLength);
            check(got->exposureStartNs == r.frameEndNs() - readout - 1200000000LL);
        }
        inFlight.pop_front();
        ++done;
    }
    check(done == 4);
    check(own);
    check(r.binder.missed() == 0);
}

/* A long manual still with no request right behind it -- the app sends
 * the next one only later: the frames after it go back to the preview's
 * settings, not on through its seconds-long exposure. */
void testOneShotSettings() {
    Rig r;
    const SensorConfig::FrameTiming t = r.cfg.timingForExposure(1200000);
    DelayedControls::Batch longShot = settings(r.cfg, 1200000);
    longShot.val[DelayedControls::FRAME_LENGTH] = t.frameLen;
    longShot.val[DelayedControls::LINE_LENGTH]  = t.lineLength;

    const DelayedControls::Batch preview = settings(r.cfg, 8000);
    std::shared_ptr<FrameBinder::Ticket> p = r.binder.request(preview, false);
    std::shared_ptr<FrameBinder::Ticket> shot = r.binder.request(longShot, true);
    for (uint32_t k = 0; k < shot->frame + 6; ++k) r.frame();

    check(shot->bayer && same(shot->inEffect, longShot));
    for (uint32_t k = shot->frame + 1; k < shot->frame + 6; ++k)
        check(same(r.ran[k], preview));
}

/* The capture misses the frame a request asked for, and the next frame
 * ran on another request's settings: the request is lost. So is the one
 * whose write the missed frame's tick would have made (kDelay frames
 * on). Every other request still gets its own frame -- a miss never
 * shifts them -- and the numbers stay right after. */
void testMissedFrameLost() {
    Rig r;
    std::deque<InFlight> inFlight;
    for (int i = 0; i < 6; ++i) {
        const DelayedControls::Batch s = settings(r.cfg, 3000 + 100 * i);
        inFlight.push_back({r.binder.request(s, false), s});
    }
    const std::shared_ptr<FrameBinder::Ticket> missing = inFlight[2].ticket;
    const std::shared_ptr<FrameBinder::Ticket> unwritten = inFlight[5].ticket;
    check(unwritten->frame == missing->frame + kDelay);

    bool own = true;
    int got = 0;
    for (uint32_t k = 0; k < 20 && !inFlight.empty(); ++k) {
        const std::optional<FrameBinder::Arrival> a = r.frame(k != missing->frame);
        if (!a) continue;
        for (const std::shared_ptr<FrameBinder::Ticket> &t : a->settled) {
            check(t == inFlight.front().ticket);
            if (t == missing || t == unwritten) {
                check(t->lost && !t->bayer);
            } else {
                own = own && !t->lost && t->sensorFrame == t->frame
                          && same(r.ran[t->sensorFrame], inFlight.front().wanted);
                ++got;
            }
            inFlight.pop_front();
        }
    }
    check(inFlight.empty());
    check(got == 4);
    check(own);
    check(r.binder.missed() == 1);
}

/* The missed frame's request wanted what the next frame ran on -- the
 * same exposure as the request after it, as auto AE's requests often
 * do: both get that frame. */
void testMissedFrameShared() {
    Rig r;
    const DelayedControls::Batch s = settings(r.cfg, 4000);
    std::shared_ptr<FrameBinder::Ticket> a1 = r.binder.request(s, false);
    std::shared_ptr<FrameBinder::Ticket> a2 = r.binder.request(s, false);

    std::vector<std::shared_ptr<FrameBinder::Ticket>> settled;
    for (uint32_t k = 0; k <= a2->frame; ++k) {
        const std::optional<FrameBinder::Arrival> a = r.frame(k != a1->frame);
        if (a)
            settled.insert(settled.end(), a->settled.begin(), a->settled.end());
    }
    check(settled.size() == 2 && settled[0] == a1 && settled[1] == a2);
    check(!a1->lost && a1->bayer && a1->sensorFrame == a2->frame);
    check(!a2->lost && a2->bayer && a2->sensorFrame == a2->frame);
    check(same(a1->inEffect, s) && same(a2->inEffect, s));
}

/* A request let go before its frame: the frame goes back with its
 * lease, and the request after it gets its own frame. */
void testRequestLetGo() {
    Rig r;
    std::shared_ptr<FrameBinder::Ticket> dropped =
        r.binder.request(settings(r.cfg, 5000), false);
    const DelayedControls::Batch s = settings(r.cfg, 6000);
    std::shared_ptr<FrameBinder::Ticket> kept = r.binder.request(s, false);
    const uint32_t droppedFrame = dropped->frame;
    dropped.reset();

    for (uint32_t k = 0; k <= kept->frame; ++k) {
        const int before = r.slots.returned;
        const std::optional<FrameBinder::Arrival> a = r.frame();
        if (k == droppedFrame)
            check(a->settled.empty() && r.slots.returned == before + 1);
    }
    check(kept->bayer.has_value());
    check(kept->sensorFrame == kept->frame);
    check(same(kept->inEffect, s));
}

/* A long exposure given up before its write: it comes off its frame,
 * which runs on what was filed before, and the request after it still
 * gets its own frame. */
void testCancel() {
    Rig r;
    const SensorConfig::FrameTiming t = r.cfg.timingForExposure(1200000);
    DelayedControls::Batch longShot = settings(r.cfg, 1200000);
    longShot.val[DelayedControls::FRAME_LENGTH] = t.frameLen;
    longShot.val[DelayedControls::LINE_LENGTH]  = t.lineLength;

    std::shared_ptr<FrameBinder::Ticket> gone = r.binder.request(longShot, true);
    const DelayedControls::Batch s = settings(r.cfg, 7000);
    std::shared_ptr<FrameBinder::Ticket> kept = r.binder.request(s, false);
    const uint32_t goneFrame = gone->frame;
    r.binder.cancel(*gone);
    gone.reset();

    for (uint32_t k = 0; k <= kept->frame; ++k) r.frame();
    check(same(r.ran[goneFrame], r.cfg.configureDefaults()));
    check(kept->bayer && kept->sensorFrame == kept->frame);
    check(same(kept->inEffect, s));
}

} // namespace

int main() {
    testInFlight(1);
    testInFlight(2);
    testInFlight(4);
    testInFlight(8);
    testNextFrameStart();
    testLongFrame();
    testOneShotSettings();
    testMissedFrameLost();
    testMissedFrameShared();
    testRequestLetGo();
    testCancel();
    return finish("frame_binder_test");
}
