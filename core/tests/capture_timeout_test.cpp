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

/* Host test for CaptureTimeout: the VI's frame wait follows the frame
 * lengths written -- up at once, down only once the longer frames are
 * through. Run by core/tests/run_host_tests.sh. */

#include "CaptureTimeout.h"

#include <vector>

#include "Check.h"

using android::CaptureTimeout;
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
    CaptureTimeout::Writer writer() {
        return [this](int32_t us) { writes.push_back(us); };
    }
};

/* Twice the period and some slack; never below the floor. */
void testFormula() {
    check(CaptureTimeout::timeoutUsFor(kFrameNs) == CaptureTimeout::kFloorUs);
    check(CaptureTimeout::timeoutUsFor(kLongNs) ==
          2 * 1000000 + CaptureTimeout::kSlackUs);
}

/* A new stream writes the timeout for its frame once; the same frame
 * again writes nothing. */
void testReset() {
    Vi vi;
    CaptureTimeout t(vi.writer(), kDelay, kMinUs, kMaxUs);
    t.reset(kFrameNs);
    check(vi.writes.size() == 1 && vi.writes[0] == CaptureTimeout::kFloorUs);
    for (int i = 0; i < 20; ++i) t.frame(kFrameNs);
    check(vi.writes.size() == 1);
}

/* A long frame raises the timeout on its own write -- before it can
 * reach the sensor -- and the timeout stays up for every frame that
 * can still be the long one: the control delay plus the latch slack. */
void testRaiseThenLower() {
    Vi vi;
    CaptureTimeout t(vi.writer(), kDelay, kMinUs, kMaxUs);
    t.reset(kFrameNs);
    const int32_t longUs = CaptureTimeout::timeoutUsFor(kLongNs);

    t.frame(kLongNs);
    check(t.current() == longUs);

    const int held = kDelay + CaptureTimeout::kLatchSlack;
    for (int i = 0; i < held; ++i) {
        t.frame(kFrameNs);
        check(t.current() == longUs);
    }
    t.frame(kFrameNs);
    check(t.current() == CaptureTimeout::kFloorUs);
    check(vi.writes.size() == 3);
}

/* Never outside the range the VI takes. */
void testClamp() {
    Vi vi;
    CaptureTimeout t(vi.writer(), kDelay, kMinUs, 1500000);
    t.reset(kLongNs);
    check(t.current() == 1500000);
}

} // namespace

int main() {
    testFormula();
    testReset();
    testRaiseThenLower();
    testClamp();
    return finish("capture_timeout_test");
}
