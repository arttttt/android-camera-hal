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

/* Host test for DelayedControls and ExposureTarget: what requests file
 * for a sensor frame, what each frame's tick writes, what was in effect
 * on a frame, and the IPA's decision requests in auto AE take. Run by
 * core/tests/run_host_tests.sh. */

#include <cstdio>

#include "sensor/DelayedControls.h"
#include "sensor/ExposureTarget.h"
#include "Check.h"

using android::DelayedControls;
using android::ExposureTarget;

using mocha::camera::test::check;
using mocha::camera::test::finish;

namespace {

using Id = DelayedControls::ControlId;

DelayedControls::Config config(int delay = 3) {
    DelayedControls::Config c;
    for (int &d : c.delay) d = delay;
    return c;
}

DelayedControls::Batch batch(int32_t exposure, int32_t gain) {
    DelayedControls::Batch b;
    b.has[DelayedControls::EXPOSURE] = true;
    b.val[DelayedControls::EXPOSURE] = exposure;
    b.has[DelayedControls::GAIN]     = true;
    b.val[DelayedControls::GAIN]     = gain;
    return b;
}

/* What configure wrote: every control set. */
DelayedControls::Batch initial() {
    DelayedControls::Batch b = batch(33000, 256);
    b.has[DelayedControls::FRAME_LENGTH] = true;
    b.val[DelayedControls::FRAME_LENGTH] = 2564;
    b.has[DelayedControls::LINE_LENGTH]  = true;
    b.val[DelayedControls::LINE_LENGTH]  = 3440;
    return b;
}

bool same(const DelayedControls::Batch &a, const DelayedControls::Batch &b) {
    for (int id = 0; id < DelayedControls::COUNT; ++id)
        if (b.has[id] && a.val[id] != b.val[id]) return false;
    return true;
}

int32_t exposureOn(const DelayedControls &dc, uint32_t seq) {
    return dc.inEffect(seq).val[DelayedControls::EXPOSURE];
}

/* Before any frame every frame runs on what configure wrote, and the
 * first frame a request can get is the delay's. */
void testStart() {
    DelayedControls dc(config());
    dc.reset(initial());
    check(dc.firstFileable() == 3);
    const DelayedControls::Batch b = dc.inEffect(0);
    for (int id = 0; id < DelayedControls::COUNT; ++id)
        check(b.has[id] && b.val[id] == initial().val[id]);
}

/* Filed for frame 3, written as frame 0 arrives, in effect on 3 and on;
 * frames 0..2 ran on the initial values. A tick with nothing new keeps
 * the values and says nothing changed. */
void testFiledFrameWrittenAtItsTick() {
    DelayedControls dc(config());
    dc.reset(initial());
    check(dc.file(3, batch(1000, 512)));

    bool changed = false;
    const DelayedControls::Batch w = dc.tick(0, &changed);
    check(changed);
    check(w.val[DelayedControls::EXPOSURE] == 1000 && w.val[DelayedControls::GAIN] == 512);
    check(w.has[DelayedControls::FRAME_LENGTH] && w.val[DelayedControls::FRAME_LENGTH] == 2564);
    check(exposureOn(dc, 2) == 33000);
    check(exposureOn(dc, 3) == 1000);

    dc.tick(1, &changed);
    check(!changed);
    check(exposureOn(dc, 4) == 1000);
}

/* A frame already being written cannot be filed for any more. */
void testTooLate() {
    DelayedControls dc(config());
    dc.reset(initial());
    dc.tick(0, nullptr);
    check(dc.firstFileable() == 4);
    check(!dc.file(3, batch(1, 1)));
    check(dc.file(4, batch(2, 2)));
}

/* Requests filed for consecutive frames -- several arriving during one
 * long frame -- are written one per tick, each on its own frame. */
void testOnePerFrame() {
    DelayedControls dc(config());
    dc.reset(initial());
    for (uint32_t k = 0; k < 7; ++k) dc.tick(k, nullptr);
    check(dc.file(10, batch(100, 256)));
    check(dc.file(11, batch(200, 256)));
    check(dc.file(12, batch(300, 256)));
    dc.tick(7, nullptr);
    dc.tick(8, nullptr);
    dc.tick(9, nullptr);
    check(exposureOn(dc, 9) == 33000);
    check(exposureOn(dc, 10) == 100);
    check(exposureOn(dc, 11) == 200);
    check(exposureOn(dc, 12) == 300);
}

/* The capture missed frame 7: its tick never came, and what was filed
 * for frame 10 is overtaken by frame 11's at tick 8. The record says
 * frame 10 ran on the old values. */
void testMissedTick() {
    DelayedControls dc(config());
    dc.reset(initial());
    for (uint32_t k = 0; k <= 6; ++k) dc.tick(k, nullptr);
    dc.file(10, batch(100, 256));
    dc.file(11, batch(200, 256));
    dc.tick(8, nullptr);
    check(exposureOn(dc, 10) == 33000);
    check(exposureOn(dc, 11) == 200);
}

/* A request let go before its frame was written: the frame runs on what
 * was filed before it. */
void testUnfile() {
    DelayedControls dc(config());
    dc.reset(initial());
    dc.file(3, batch(100, 256));
    dc.file(4, batch(200, 256));
    dc.unfile(3);
    dc.tick(0, nullptr);
    dc.tick(1, nullptr);
    check(exposureOn(dc, 3) == 33000);
    check(exposureOn(dc, 4) == 200);
}

/* Controls with different delays, as the sensors have them (written in
 * frame k: frame length in effect on k + 1, exposure and gain on
 * k + 2). A request's controls are written together, in the tick its
 * slowest one needs; its frame length lands on the frame before its
 * own, the one that gives its exposure its time. inEffect() is what
 * each frame ran on; inEffectFor() what the request's frame was given. */
void testPerControlDelay() {
    DelayedControls::Config c;
    c.delay[DelayedControls::EXPOSURE]     = 2;
    c.delay[DelayedControls::GAIN]         = 2;
    c.delay[DelayedControls::FRAME_LENGTH] = 1;
    c.delay[DelayedControls::LINE_LENGTH]  = 1;
    DelayedControls dc(c);
    dc.reset(initial());
    check(dc.firstFileable() == 2);
    check(dc.frameOf(5, DelayedControls::EXPOSURE) == 5);
    check(dc.frameOf(5, DelayedControls::FRAME_LENGTH) == 4);

    DelayedControls::Batch req = batch(100000, 512);
    req.has[DelayedControls::FRAME_LENGTH] = true;
    req.val[DelayedControls::FRAME_LENGTH] = 9000;
    check(dc.file(5, req));
    check(dc.planned(5).val[DelayedControls::FRAME_LENGTH] == 9000);

    bool changed = false;
    for (uint32_t k = 0; k < 3; ++k) {
        dc.tick(k, &changed);
        check(!changed);
    }
    /* One tick writes all of it. */
    const DelayedControls::Batch w = dc.tick(3, &changed);
    check(changed);
    check(w.val[DelayedControls::EXPOSURE] == 100000);
    check(w.val[DelayedControls::GAIN] == 512);
    check(w.val[DelayedControls::FRAME_LENGTH] == 9000);

    check(dc.inEffect(3).val[DelayedControls::FRAME_LENGTH] == 2564);
    check(dc.inEffect(4).val[DelayedControls::FRAME_LENGTH] == 9000);
    check(dc.inEffect(4).val[DelayedControls::EXPOSURE] == 33000);
    check(dc.inEffect(5).val[DelayedControls::EXPOSURE] == 100000);
    check(same(dc.inEffectFor(5), req));
}

/* The first frames of a stream, with the sensors' delays: a request's
 * frame length lands on the frame before its own, and forgetting what
 * came before frame 0 forgets nothing it needs (no wrap below 0). */
void testEarlyFrames() {
    DelayedControls::Config c;
    c.delay[DelayedControls::EXPOSURE]     = 2;
    c.delay[DelayedControls::GAIN]         = 2;
    c.delay[DelayedControls::FRAME_LENGTH] = 1;
    c.delay[DelayedControls::LINE_LENGTH]  = 1;
    DelayedControls dc(c);
    dc.reset(initial());
    DelayedControls::Batch a = batch(100, 256), b = batch(200, 256);
    a.has[DelayedControls::FRAME_LENGTH] = true;
    a.val[DelayedControls::FRAME_LENGTH] = 2000;
    b.has[DelayedControls::FRAME_LENGTH] = true;
    b.val[DelayedControls::FRAME_LENGTH] = 3000;
    check(dc.file(2, a) && dc.file(3, b));
    dc.tick(0, nullptr);
    dc.tick(1, nullptr);
    dc.forgetBefore(0);
    check(same(dc.inEffectFor(2), a));
    check(same(dc.inEffectFor(3), b));
    /* A frame before the stream's first runs on what configure wrote. */
    check(dc.inEffectFor(0).val[DelayedControls::FRAME_LENGTH] == 2564);
}

/* The stream's first frame to start is 1, not 0: what was filed before
 * any tick moves on by it, and is written at the ticks its new frames
 * need. */
void testRebase() {
    DelayedControls dc(config());
    dc.reset(initial());
    check(dc.file(3, batch(100, 256)));
    dc.rebase(1);
    dc.tick(1, nullptr);
    check(exposureOn(dc, 3) == 33000);
    check(exposureOn(dc, 4) == 100);
}

/* What a frame would run on as things stand: the latest filed for it or
 * before, else the last written, else the initial values. */
void testPlanned() {
    DelayedControls dc(config());
    dc.reset(initial());
    check(dc.planned(5).val[DelayedControls::EXPOSURE] == 33000);
    dc.file(4, batch(100, 256));
    dc.file(7, batch(200, 256));
    check(dc.planned(3).val[DelayedControls::EXPOSURE] == 33000);
    check(dc.planned(5).val[DelayedControls::EXPOSURE] == 100);
    check(dc.planned(9).val[DelayedControls::EXPOSURE] == 200);
    dc.tick(1, nullptr);   /* writes frame 4's */
    check(dc.planned(5).val[DelayedControls::EXPOSURE] == 100);
}

/* reset() forgets what was filed and written. */
void testReset() {
    DelayedControls dc(config());
    dc.reset(initial());
    dc.file(3, batch(100, 10));
    dc.tick(0, nullptr);
    dc.reset(initial());
    check(dc.firstFileable() == 3);
    check(exposureOn(dc, 3) == 33000);
    bool changed = true;
    dc.tick(0, &changed);
    check(!changed);
}

/* The target is nothing until the IPA decides; then each control keeps
 * its last decision, whichever batch it came in, until reset. */
void testExposureTarget() {
    ExposureTarget t;
    check(!t.latest());

    t.publish(batch(1000, 256));
    DelayedControls::Batch fl;
    fl.has[DelayedControls::FRAME_LENGTH] = true;
    fl.val[DelayedControls::FRAME_LENGTH] = 2564;
    t.publish(fl);
    t.publish(DelayedControls::Batch{});   /* nothing decided: no change */

    const std::optional<DelayedControls::Batch> b = t.latest();
    check(b && b->has[DelayedControls::EXPOSURE] && b->val[DelayedControls::EXPOSURE] == 1000);
    check(b && b->has[DelayedControls::GAIN] && b->val[DelayedControls::GAIN] == 256);
    check(b && b->has[DelayedControls::FRAME_LENGTH] && b->val[DelayedControls::FRAME_LENGTH] == 2564);
    check(b && !b->has[DelayedControls::LINE_LENGTH]);

    t.reset();
    check(!t.latest());
}

} // namespace

int main() {
    testStart();
    testFiledFrameWrittenAtItsTick();
    testTooLate();
    testOnePerFrame();
    testMissedTick();
    testUnfile();
    testPerControlDelay();
    testPlanned();
    testEarlyFrames();
    testRebase();
    testReset();
    testExposureTarget();
    return finish("delayed_controls_test");
}
