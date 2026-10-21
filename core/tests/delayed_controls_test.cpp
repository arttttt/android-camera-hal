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

/* Host test for DelayedControls: what the IPA files for a frame, what
 * ApplySettings commits, and what result metadata reads back as in
 * effect. Run by core/tests/run_host_tests.sh. */

#include <cstdio>

#include "sensor/DelayedControls.h"
#include "Check.h"

using android::DelayedControls;

using mocha::camera::test::check;
using mocha::camera::test::finish;

namespace {

DelayedControls::Config config() {
    DelayedControls::Config c;
    c.delay[DelayedControls::EXPOSURE]        = 2;
    c.delay[DelayedControls::GAIN]            = 2;
    c.defaultValue[DelayedControls::EXPOSURE] = 0;
    c.defaultValue[DelayedControls::GAIN]     = 0;
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

/* The IPA files a decision for frame 12; ApplySettings finds it there
 * when frame 12 is written. */
void testPendingWrite() {
    DelayedControls dc(config());
    check(!dc.pendingWrite(12).has[DelayedControls::EXPOSURE]);
    dc.push(12, batch(1000, 256));
    const DelayedControls::Batch b = dc.pendingWrite(12);
    check(b.has[DelayedControls::EXPOSURE] && b.val[DelayedControls::EXPOSURE] == 1000);
    check(b.has[DelayedControls::GAIN] && b.val[DelayedControls::GAIN] == 256);
    check(!dc.pendingWrite(13).has[DelayedControls::EXPOSURE]);
}

/* What was written for frame N is in effect on frame N + delay; before
 * any write the caller's fallback (the configured defaults) is. */
void testInEffect() {
    DelayedControls dc(config());
    const int32_t fallback[DelayedControls::COUNT] = {33000, 100};
    DelayedControls::Batch b = dc.applyControls(5, fallback);
    check(b.val[DelayedControls::EXPOSURE] == 33000 && b.val[DelayedControls::GAIN] == 100);

    dc.commit(5, batch(2000, 300));
    b = dc.applyControls(7, fallback);
    check(b.val[DelayedControls::EXPOSURE] == 2000 && b.val[DelayedControls::GAIN] == 300);
    b = dc.applyControls(6, fallback);
    check(b.val[DelayedControls::EXPOSURE] == 33000);
}

/* A late IPA push for a frame already written does not rewrite the
 * record: result metadata reports what the sensor really got. */
void testSealAgainstLatePush() {
    DelayedControls dc(config());
    dc.commit(9, batch(4000, 512));
    dc.push(9, batch(9999, 999));
    const DelayedControls::Batch b = dc.applyControls(11);
    check(b.val[DelayedControls::EXPOSURE] == 4000 && b.val[DelayedControls::GAIN] == 512);
}

/* A push for a later frame in the same ring slot replaces a stale cell;
 * the stale frame then reads as never written. */
void testRingWrap() {
    DelayedControls dc(config());
    dc.commit(3, batch(100, 10));
    dc.push(3 + 16, batch(200, 20));
    const int32_t fallback[DelayedControls::COUNT] = {7, 8};
    DelayedControls::Batch b = dc.applyControls(3 + 2, fallback);
    check(b.val[DelayedControls::EXPOSURE] == 7);
    b = dc.pendingWrite(3 + 16);
    check(b.val[DelayedControls::EXPOSURE] == 200);
}

/* reset() forgets everything. */
void testReset() {
    DelayedControls dc(config());
    dc.commit(1, batch(100, 10));
    dc.reset();
    check(dc.applyControls(3).val[DelayedControls::EXPOSURE] == 0);
    check(!dc.pendingWrite(1).has[DelayedControls::EXPOSURE]);
}

} // namespace

int main() {
    testPendingWrite();
    testInEffect();
    testSealAgainstLatePush();
    testRingWrap();
    testReset();
    return finish("delayed_controls_test");
}
