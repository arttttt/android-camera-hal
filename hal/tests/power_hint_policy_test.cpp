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

/* Host test for PowerHintPolicy: the levels by pixel rate, the shot's
 * length, and the hints a session's events send. */

#include "Check.h"
#include "PowerHintPolicy.h"

using android::PowerHintPolicy;
using mocha::camera::test::check;
using mocha::camera::test::finish;

namespace {

using Hint   = PowerHintPolicy::Hint;
using Action = PowerHintPolicy::Action;
using Kind   = Action::Kind;

constexpr uint64_t k720p30  = 1280ull * 720 * 30;
constexpr uint64_t k720p60  = 1280ull * 720 * 60;
constexpr uint64_t k1080p30 = 1920ull * 1080 * 30;

void testLevels() {
    check(PowerHintPolicy::levelFor(k720p30) == Hint::StreamingLow);
    /* A mode a little fast stays in its level. */
    check(PowerHintPolicy::levelFor(k720p30 * 101 / 100) == Hint::StreamingLow);
    check(PowerHintPolicy::levelFor(k720p60) == Hint::StreamingMid);
    check(PowerHintPolicy::levelFor(k1080p30) == Hint::StreamingMid);
    check(PowerHintPolicy::levelFor(k1080p30 * 101 / 100) == Hint::StreamingMid);
    check(PowerHintPolicy::levelFor(1920ull * 1080 * 60) == Hint::StreamingHigh);
}

void testShot() {
    check(PowerHintPolicy::shotMsFor(1920ull * 1080) == 300);
    check(PowerHintPolicy::shotMsFor(1280ull * 720) == 300);
    check(PowerHintPolicy::shotMsFor(1920ull * 1080 * 2) == 600);
    check(PowerHintPolicy::shotMsFor(3280ull * 2464) == 1000);
}

void testSession() {
    PowerHintPolicy p;
    check(p.opening() == std::vector<Action>{{Kind::Boost, Hint::Launch, 500}});

    /* Nothing streams yet: stopping sends nothing. */
    check(p.stopped().empty());

    check(p.streaming(k1080p30) == std::vector<Action>{{Kind::ModeOn, Hint::StreamingMid}});
    check(p.level() == Hint::StreamingMid);
    /* The same level again: nothing. */
    check(p.streaming(k720p60).empty());

    check(p.stillCapture(1920ull * 1080) ==
          std::vector<Action>{{Kind::Boost, Hint::Shot, 300}});

    /* A new level: the old one off first. */
    check(p.streaming(k720p30) == (std::vector<Action>{{Kind::ModeOff, Hint::StreamingMid},
                                                       {Kind::ModeOn, Hint::StreamingLow}}));

    check(p.stopped() == std::vector<Action>{{Kind::ModeOff, Hint::StreamingLow}});
    check(!p.level());
    check(p.stopped().empty());
}

} /* namespace */

int main() {
    testLevels();
    testShot();
    testSession();
    return finish("power_hint_policy_test");
}
