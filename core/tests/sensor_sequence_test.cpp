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

/* Host test for SensorSequence: the sensor's frame numbers from the
 * driver's buffer count and times, across frames the driver missed,
 * damaged frames it counted, long frames and timing jitter. Run by
 * core/tests/run_host_tests.sh. */

#include <cstdint>

#include "sensor/SensorSequence.h"
#include "Check.h"

using android::SensorSequence;

using mocha::camera::test::check;
using mocha::camera::test::finish;

namespace {

constexpr int64_t kPeriod = 33333333;   /* 30 fps */

int64_t steady(uint32_t) { return kPeriod; }

/* The first frame is 0 whatever the driver's count; then one a frame. */
void testSteady() {
    SensorSequence s;
    check(s.next(57, 1000000000, steady) == 0);
    int64_t t = 1000000000;
    for (uint32_t i = 1; i <= 10; ++i) {
        t += kPeriod;
        check(s.next(57 + i, t, steady) == i);
    }
    check(s.uncounted() == 0);
}

/* The VI had no buffer for two frames: the driver counts one, the time
 * says three. */
void testMissedFrames() {
    SensorSequence s;
    s.next(0, 0, steady);
    check(s.next(1, 3 * kPeriod, steady) == 3);
    check(s.uncounted() == 2);
    check(s.next(2, 4 * kPeriod, steady) == 4);
}

/* A damaged frame the HAL gave back is counted by the driver: two
 * counted, two passed, nothing missed. */
void testDamagedFrameCounted() {
    SensorSequence s;
    s.next(10, 0, steady);
    check(s.next(12, 2 * kPeriod, steady) == 2);
    check(s.uncounted() == 0);
}

/* A 1.2 s frame is one frame, not 36: periods come from what was in
 * effect on each frame. */
void testLongFrame() {
    const int64_t longPeriod = 1200000000;
    const auto periodOf = [&](uint32_t seq) {
        return seq == 3 ? longPeriod : kPeriod;
    };
    SensorSequence s;
    s.next(0, 0, periodOf);
    s.next(1, kPeriod, periodOf);
    s.next(2, 2 * kPeriod, periodOf);
    check(s.next(3, 2 * kPeriod + longPeriod, periodOf) == 3);
    check(s.next(4, 3 * kPeriod + longPeriod, periodOf) == 4);
    /* The frame after the long one missed: the time holds two short
     * frames. */
    check(s.next(5, 5 * kPeriod + longPeriod, periodOf) == 6);
    check(s.uncounted() == 1);
}

/* Interrupt latency moves a buffer's time by far less than half a
 * frame: no frame is invented or lost. */
void testJitter() {
    SensorSequence s;
    s.next(0, 0, steady);
    check(s.next(1, kPeriod + kPeriod / 5, steady) == 1);
    check(s.next(2, 2 * kPeriod - kPeriod / 5, steady) == 2);
    check(s.uncounted() == 0);
}

/* A period nobody can tell stops the search at the driver's count. */
void testUnknownPeriod() {
    SensorSequence s;
    s.next(0, 0, [](uint32_t) { return int64_t(0); });
    check(s.next(1, 10 * kPeriod, [](uint32_t) { return int64_t(0); }) == 1);
}

/* reset() starts the next stream from 0. */
void testReset() {
    SensorSequence s;
    s.next(0, 0, steady);
    s.next(1, 3 * kPeriod, steady);
    s.reset();
    check(s.next(400, 99 * kPeriod, steady) == 0);
    check(s.uncounted() == 0);
}

} // namespace

int main() {
    testSteady();
    testMissedFrames();
    testDamagedFrameCounted();
    testLongFrame();
    testJitter();
    testUnknownPeriod();
    testReset();
    return finish("sensor_sequence_test");
}
