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

/* The sensors' delays (22.10.2026, FRAME_START traces on both): written
 * in frame k, the frame and line length are in effect on k + 1, the
 * exposure and gain on k + 2. */
constexpr int kExposureDelay = 2;
constexpr int kLengthDelay   = 1;
constexpr int kDelay         = kExposureDelay;   /* the slowest */
constexpr uint32_t kLines    = 1080;

DelayedControls::Config sensorDelays() {
    DelayedControls::Config c;
    c.delay[DelayedControls::EXPOSURE]     = kExposureDelay;
    c.delay[DelayedControls::GAIN]         = kExposureDelay;
    c.delay[DelayedControls::FRAME_LENGTH] = kLengthDelay;
    c.delay[DelayedControls::LINE_LENGTH]  = kLengthDelay;
    return c;
}

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
 * frame: it runs on what is in effect by then, control by control; its
 * start is reported -- unless the capture misses the frame -- and the
 * binder's write, made inside it, takes effect each control's delay
 * on; it lasts what its frame and line length make it, or its exposure
 * where that is longer, and arrives, numbered as the driver numbers
 * frames. A frame whose line length is new to it runs without a start
 * (SensorConfig::lineLengthHidesFrame, as the IMX179): no write is made
 * in it, nothing arrives, and the driver does not count it. */
class Rig {
public:
    SensorConfig    cfg = imx179();
    DelayedControls dc{sensorDelays()};
    FrameBinder     binder{&dc, &cfg};
    Slots           slots;

    /* What each sensor frame really ran on. */
    std::vector<DelayedControls::Batch> ran;

    /* `first`: the number the driver gives the stream's first frame. */
    explicit Rig(uint32_t first = 0) : first(first) {
        dc.reset(cfg.configureDefaults());
        binder.reset(kLines);
        running = cfg.configureDefaults();
    }

    std::optional<FrameBinder::Arrival> frame(bool captured = true) {
        const uint32_t k = static_cast<uint32_t>(ran.size());
        for (int id = 0; id < DelayedControls::COUNT; ++id) {
            auto w = writes[id].find(k);
            if (w != writes[id].end()) running.val[id] = w->second;
        }
        const bool hidden = cfg.lineLengthHidesFrame && k > 0 &&
            running.val[DelayedControls::LINE_LENGTH] !=
                ran.back().val[DelayedControls::LINE_LENGTH];
        ran.push_back(running);
        endNs += cfg.frameSpanNs(running);
        if (hidden) {
            ++hiddenFrames;
            return std::nullopt;
        }
        const uint32_t seq = k - hiddenFrames + first;
        if (!captured)
            return std::nullopt;

        const FrameBinder::Start s = binder.start(seq);
        if (s.ticked && s.changed) {
            const DelayedControls::Config d = sensorDelays();
            for (int id = 0; id < DelayedControls::COUNT; ++id)
                writes[id][k + d.delay[id]] = s.values.val[id];
        }

        BayerFrame f;
        f.slot        = static_cast<int>(k % 8);
        f.sequence    = seq;
        f.timestampNs = endNs;
        return binder.arrive(BayerLease(f, 0, &slots));
    }

    int64_t frameEndNs() const { return endNs; }

private:
    uint32_t first;
    std::map<uint32_t, int32_t> writes[DelayedControls::COUNT];
    DelayedControls::Batch running;
    uint32_t hiddenFrames = 0;
    int64_t  endNs = 1000000000;
};

/* What a request on frame `seq` ran on, each control on the frame the
 * sensor's delays put it on. */
DelayedControls::Batch ranFor(const Rig &r, uint32_t seq) {
    const DelayedControls::Config d = sensorDelays();
    DelayedControls::Batch b;
    for (int id = 0; id < DelayedControls::COUNT; ++id) {
        b.has[id] = true;
        b.val[id] = r.ran[seq - (kDelay - d.delay[id])].val[id];
    }
    return b;
}

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
        own     = own && same(ranFor(r, t->sensorFrame), front.wanted)
                      && same(t->inEffect, r.ran[t->sensorFrame]);
        asked   = asked && !t->lost && t->sensorFrame == t->frame;
        lastFrame = t->sensorFrame;
        inFlight.pop_front();
        ++done;
        send();
    }
    check(ordered);
    check(own);
    check(asked);
    check(r.binder.missedFrames() == 0 && r.binder.missedStarts() == 0);

    /* A request sent as a frame arrives can get the frame kDelay + 1
     * on; n in flight fill that many of each kDelay + 1 frames, up to
     * all of them. */
    const size_t perCycle = n < (size_t)kDelay + 1 ? n : (size_t)kDelay + 1;
    const int expected = (int)(kFrames * perCycle / (kDelay + 1));
    check(done >= expected - (int)(2 * (kDelay + 1)));
    check(done <= expected + 1);
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
                  && same(ranFor(r, got->sensorFrame), inFlight.front().wanted)
                  && same(got->inEffect, r.ran[got->sensorFrame]);
        if (done == 1) {
            /* The frame before it is the long one -- it gives the
             * exposure its time -- and its own reads out at its own
             * line length. */
            const DelayedControls::Batch &before = r.ran[got->sensorFrame - 1];
            const int64_t period = r.cfg.frameDurationNs(
                    before.val[DelayedControls::FRAME_LENGTH],
                    before.val[DelayedControls::LINE_LENGTH]);
            check(period > 1190000000 && period < 1260000000);
            /* Its exposure began 1.2 s and a read-out before its end. */
            const int64_t readout = (int64_t)kLines * r.cfg.lineTimeNsAt(
                    r.ran[got->sensorFrame].val[DelayedControls::LINE_LENGTH]);
            check(got->exposureStartNs == r.frameEndNs() - readout - 1200000000LL);
        }
        inFlight.pop_front();
        ++done;
    }
    check(done == 4);
    check(own);
    check(r.binder.missedFrames() == 0 && r.binder.missedStarts() == 0);
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

    check(shot->bayer && same(ranFor(r, shot->sensorFrame), longShot));
    check(same(shot->inEffect, r.ran[shot->sensorFrame]));
    /* The frame right after it is the preview's frame and line length,
     * and runs hidden on the still's exposure, the slower control; from
     * the next one on, all of it is the preview's. */
    const DelayedControls::Batch &after = r.ran[shot->frame + 1];
    check(after.val[DelayedControls::FRAME_LENGTH] ==
              preview.val[DelayedControls::FRAME_LENGTH] &&
          after.val[DelayedControls::LINE_LENGTH] ==
              preview.val[DelayedControls::LINE_LENGTH]);
    for (uint32_t k = shot->frame + 2; k < shot->frame + 6; ++k)
        check(same(r.ran[k], preview));
}

/* A long still whose line length is not the preview's, among preview
 * requests sent at the framework's pace: the sensor hides the frame
 * before it and the frame after it. The still gets its own frame,
 * every preview request gets one that ran on its settings -- none is
 * given a hidden frame, or one whose write a hidden frame would have
 * made -- and the driver's numbers, one short past each hidden frame,
 * are no frame or start missed. From the start the still's write goes
 * out on, the VI's wait covers the frame and the hidden one after it. */
void testHiddenFrames() {
    Rig r;
    const SensorConfig::FrameTiming t = r.cfg.timingForExposure(1200000);
    check(t.lineLength != r.cfg.lineLengthDefault);
    DelayedControls::Batch longShot = settings(r.cfg, 1200000);
    longShot.val[DelayedControls::FRAME_LENGTH] = t.frameLen;
    longShot.val[DelayedControls::LINE_LENGTH]  = t.lineLength;
    const int64_t longPeriod = r.cfg.frameDurationNs(t.frameLen, t.lineLength);

    std::deque<InFlight> inFlight;
    int next = 0;
    std::shared_ptr<FrameBinder::Ticket> shot;
    const auto send = [&] {
        const bool still = next == 6;
        const DelayedControls::Batch s = still ? longShot
                                               : settings(r.cfg, 1000 + 10 * next);
        ++next;
        inFlight.push_back({r.binder.request(s, still), s});
        if (still) shot = inFlight.back().ticket;
    };
    for (int i = 0; i < 4; ++i) send();

    bool own = true, waitCovers = true;
    int done = 0;
    for (uint32_t k = 0; k < 60; ++k) {
        const std::optional<FrameBinder::Arrival> a = r.frame();
        if (!a || a->settled.empty()) continue;
        for (const std::shared_ptr<FrameBinder::Ticket> &got : a->settled) {
            own = own && !got->lost && got == inFlight.front().ticket
                      && got->sensorFrame == got->frame
                      && same(ranFor(r, got->sensorFrame), inFlight.front().wanted)
                      && same(got->inEffect, r.ran[got->sensorFrame]);
            inFlight.pop_front();
            ++done;
            send();
        }
    }
    check(own);
    check(done > 30);
    check(shot && shot->bayer && shot->inEffect.val[DelayedControls::EXPOSURE] == 1200000);
    /* The hidden frames ran, and nothing came for them. */
    check(r.ran[shot->frame - 1].val[DelayedControls::LINE_LENGTH] == t.lineLength);
    check(r.ran[shot->frame + 1].val[DelayedControls::LINE_LENGTH] ==
          r.cfg.lineLengthDefault);
    check(r.binder.missedFrames() == 0 && r.binder.missedStarts() == 0);

    /* The VI: a binder of its own, the same requests, started by hand. */
    DelayedControls dc(sensorDelays());
    dc.reset(r.cfg.configureDefaults());
    FrameBinder binder(&dc, &r.cfg);
    binder.reset(kLines);
    binder.request(settings(r.cfg, 8000), false);
    const std::shared_ptr<FrameBinder::Ticket> s = binder.request(longShot, true);
    const uint32_t writeTick = s->frame - kDelay;
    for (uint32_t k = 0; k < writeTick; ++k) binder.start(k);
    const FrameBinder::Start w = binder.start(writeTick);
    waitCovers = w.changed && w.waitPeriodNs >= w.periodNs + longPeriod;
    check(waitCovers);
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
    const std::shared_ptr<FrameBinder::Ticket> unwritten = inFlight[2 + kDelay].ticket;
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
                          && same(ranFor(r, t->sensorFrame), inFlight.front().wanted);
                ++got;
            }
            inFlight.pop_front();
        }
    }
    check(inFlight.empty());
    check(got == 4);
    check(own);
    check(r.binder.missedFrames() == 1 && r.binder.missedStarts() == 1);
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
    check(same(ranFor(r, a2->frame), s));
    check(same(a1->inEffect, r.ran[a2->frame]) && same(a2->inEffect, r.ran[a2->frame]));
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
    check(same(ranFor(r, kept->frame), s));
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
    check(same(ranFor(r, kept->frame), s));
}

/* The driver's first frame is 1 -- a frame started between the stream's
 * counter base and the capture's first: the requests made before it
 * move on and get their own frames, numbered as the driver numbers
 * them. */
void testFirstFrameNotZero() {
    Rig r(1);
    std::deque<InFlight> inFlight;
    for (int i = 0; i < 4; ++i) {
        const DelayedControls::Batch s = settings(r.cfg, 1000 + 100 * i);
        inFlight.push_back({r.binder.request(s, false), s});
    }
    bool own = true;
    for (uint32_t k = 0; k < 12 && !inFlight.empty(); ++k) {
        const std::optional<FrameBinder::Arrival> a = r.frame();
        if (!a || a->settled.empty()) continue;
        for (const std::shared_ptr<FrameBinder::Ticket> &t : a->settled) {
            own = own && t == inFlight.front().ticket && !t->lost
                      && t->sensorFrame == t->frame
                      && same(ranFor(r, t->sensorFrame - 1), inFlight.front().wanted);
            inFlight.pop_front();
        }
    }
    check(inFlight.empty());
    check(own);
}

/* A frame number again, or a start after its own frame: given back, not
 * written on, and nothing counted as missed. */
void testOutOfOrder() {
    SensorConfig cfg = imx179();
    DelayedControls dc(sensorDelays());
    dc.reset(cfg.configureDefaults());
    FrameBinder binder(&dc, &cfg);
    binder.reset(kLines);
    Slots slots;
    const auto lease = [&](uint32_t seq) {
        BayerFrame f;
        f.slot = 0;
        f.sequence = seq;
        f.timestampNs = 1000;
        return BayerLease(f, 0, &slots);
    };
    check(binder.start(0).ticked);
    binder.arrive(lease(0));
    check(!binder.start(0).ticked);
    check(binder.arrive(lease(0)).settled.empty());
    check(slots.returned == 2);
    check(binder.missedFrames() == 0 && binder.missedStarts() == 0);
    check(binder.start(1).ticked);
}

/* The VI's wait is up a start before the write that makes a long frame:
 * a sensor may stretch the very frame a frame length is written in, and
 * the VI starts waiting on the next frame as that one starts. At the
 * start before the long still's write the wait already covers it; it
 * is not up before then. */
void testViWaitAhead() {
    SensorConfig cfg = imx179();
    DelayedControls dc(sensorDelays());
    dc.reset(cfg.configureDefaults());
    FrameBinder binder(&dc, &cfg);
    binder.reset(kLines);

    const SensorConfig::FrameTiming t = cfg.timingForExposure(1200000);
    DelayedControls::Batch longShot = settings(cfg, 1200000);
    longShot.val[DelayedControls::FRAME_LENGTH] = t.frameLen;
    longShot.val[DelayedControls::LINE_LENGTH]  = t.lineLength;
    const int64_t longPeriod = cfg.frameDurationNs(t.frameLen, t.lineLength);

    binder.request(settings(cfg, 8000), false);
    binder.request(settings(cfg, 8000), false);
    const std::shared_ptr<FrameBinder::Ticket> shot = binder.request(longShot, true);
    const uint32_t writeTick = shot->frame - kDelay;

    for (uint32_t k = 0; k < writeTick - 1; ++k)
        check(binder.start(k).waitPeriodNs < longPeriod);
    check(binder.start(writeTick - 1).waitPeriodNs >= longPeriod);
    const FrameBinder::Start w = binder.start(writeTick);
    check(w.changed && w.waitPeriodNs >= longPeriod);
}

} // namespace

int main() {
    testInFlight(1);
    testInFlight(2);
    testInFlight(4);
    testInFlight(8);
    testLongFrame();
    testOneShotSettings();
    testMissedFrameLost();
    testMissedFrameShared();
    testRequestLetGo();
    testCancel();
    testFirstFrameNotZero();
    testOutOfOrder();
    testViWaitAhead();
    testHiddenFrames();
    return finish("frame_binder_test");
}
