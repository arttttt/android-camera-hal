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

/* Host test for ViController: the VI's frame wait follows the frame
 * lengths written -- up at once, down only once the longer frames are
 * through. Run by core/tests/run_host_tests.sh. */

#include "ViController.h"

#include <vector>

#include "Check.h"

using android::ViController;
using mocha::camera::test::check;
using mocha::camera::test::finish;

namespace {

constexpr int     kDelay   = 3;
constexpr int32_t kMinUs   = 20000;
constexpr int32_t kMaxUs   = 10000000;
constexpr int64_t kFrameNs = 33000000;      /* the mode's frame */
constexpr int64_t kLongNs  = 1000000000;    /* a 1 s manual frame */

struct Vi {
    std::vector<int32_t> writes;
    ViController::Writer writer() {
        return [this](int32_t us) { writes.push_back(us); };
    }
};

/* Twice the period and some slack; never below the floor. */
void testFormula() {
    check(ViController::frameWaitUsFor(kFrameNs) == ViController::kFloorUs);
    check(ViController::frameWaitUsFor(kLongNs) ==
          2 * 1000000 + ViController::kSlackUs);
}

/* A new stream writes the wait for its frame once; the same frame
 * again writes nothing. */
void testReset() {
    Vi vi;
    ViController t(vi.writer(), kDelay, kMinUs, kMaxUs);
    t.startStream(kFrameNs);
    check(vi.writes.size() == 1 && vi.writes[0] == ViController::kFloorUs);
    for (int i = 0; i < 20; ++i) t.beforeFrame(kFrameNs);
    check(vi.writes.size() == 1);
}

/* A long frame raises the wait on its own write -- before it can
 * reach the sensor -- and the wait stays up for every frame that
 * can still be the long one: the control delay plus the latch slack. */
void testRaiseThenLower() {
    Vi vi;
    ViController t(vi.writer(), kDelay, kMinUs, kMaxUs);
    t.startStream(kFrameNs);
    const int32_t longUs = ViController::frameWaitUsFor(kLongNs);

    t.beforeFrame(kLongNs);
    check(t.frameWaitUs() == longUs);

    const int held = kDelay + ViController::kLatchSlack;
    for (int i = 0; i < held; ++i) {
        t.beforeFrame(kFrameNs);
        check(t.frameWaitUs() == longUs);
    }
    t.beforeFrame(kFrameNs);
    check(t.frameWaitUs() == ViController::kFloorUs);
    check(vi.writes.size() == 3);
}

/* Never outside the range the VI takes. */
void testClamp() {
    Vi vi;
    ViController t(vi.writer(), kDelay, kMinUs, 1500000);
    t.startStream(kLongNs);
    check(t.frameWaitUs() == 1500000);
}

} // namespace

int main() {
    testFormula();
    testReset();
    testRaiseThenLower();
    testClamp();
    return finish("vi_controller_test");
}
