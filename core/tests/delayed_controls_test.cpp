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

/* Host test for DelayedControls and ExposureTarget: what ApplySettings
 * commits, what result metadata reads back as in effect, and the IPA's
 * decision requests in auto AE take. Run by
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

DelayedControls::Config config() {
    DelayedControls::Config c;
    c.delay[DelayedControls::EXPOSURE]        = 2;
    c.delay[DelayedControls::GAIN]            = 2;
    c.delay[DelayedControls::FRAME_LENGTH]    = 2;
    c.delay[DelayedControls::LINE_LENGTH]     = 2;
    return c;
}

/* What configure wrote: every control set. */
DelayedControls::Batch fallbackOf(int32_t exposure, int32_t gain,
                                  int32_t frameLen = 0) {
    DelayedControls::Batch b;
    for (bool &h : b.has) h = true;
    b.val[DelayedControls::EXPOSURE]     = exposure;
    b.val[DelayedControls::GAIN]         = gain;
    b.val[DelayedControls::FRAME_LENGTH] = frameLen;
    return b;
}

DelayedControls::Batch batch(int32_t exposure, int32_t gain) {
    DelayedControls::Batch b;
    b.has[DelayedControls::EXPOSURE] = true;
    b.val[DelayedControls::EXPOSURE] = exposure;
    b.has[DelayedControls::GAIN]     = true;
    b.val[DelayedControls::GAIN]     = gain;
    return b;
}

/* What was written for frame N is in effect on frame N + delay; before
 * any write the caller's fallback (the configured defaults) is. */
void testInEffect() {
    DelayedControls dc(config());
    const DelayedControls::Batch fallback = fallbackOf(33000, 100);
    DelayedControls::Batch b = dc.applyControls(5, fallback);
    check(b.val[DelayedControls::EXPOSURE] == 33000 && b.val[DelayedControls::GAIN] == 100);

    dc.commit(5, batch(2000, 300));
    b = dc.applyControls(7, fallback);
    check(b.val[DelayedControls::EXPOSURE] == 2000 && b.val[DelayedControls::GAIN] == 300);
    b = dc.applyControls(6, fallback);
    check(b.val[DelayedControls::EXPOSURE] == 33000);
}

/* The frame length rides with exposure and gain: written for frame N,
 * in effect on N + delay, the frame before still on the fallback. */
void testFrameLength() {
    DelayedControls dc(config());
    DelayedControls::Batch b = batch(2000, 300);
    b.has[DelayedControls::FRAME_LENGTH] = true;
    b.val[DelayedControls::FRAME_LENGTH] = 5000;
    dc.commit(5, b);
    const DelayedControls::Batch fallback = fallbackOf(33000, 100, 2564);
    check(dc.applyControls(7, fallback).val[DelayedControls::FRAME_LENGTH] == 5000);
    check(dc.applyControls(6, fallback).val[DelayedControls::FRAME_LENGTH] == 2564);
}

/* A commit for a later frame in the same ring slot replaces a stale
 * cell; the stale frame then reads as never written. */
void testRingWrap() {
    DelayedControls dc(config());
    dc.commit(3, batch(100, 10));
    dc.commit(3 + 16, batch(200, 20));
    const DelayedControls::Batch fallback = fallbackOf(7, 8);
    DelayedControls::Batch b = dc.applyControls(3 + 2, fallback);
    check(b.val[DelayedControls::EXPOSURE] == 7);
    b = dc.applyControls(3 + 16 + 2, fallback);
    check(b.val[DelayedControls::EXPOSURE] == 200);
}

/* reset() forgets everything. */
void testReset() {
    DelayedControls dc(config());
    dc.commit(1, batch(100, 10));
    dc.reset();
    check(dc.applyControls(3, fallbackOf(7, 8)).val[DelayedControls::EXPOSURE] == 7);
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
    testInEffect();
    testFrameLength();
    testRingWrap();
    testReset();
    testExposureTarget();
    return finish("delayed_controls_test");
}
