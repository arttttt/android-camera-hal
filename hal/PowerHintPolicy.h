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

#ifndef HAL_POWER_HINT_POLICY_H
#define HAL_POWER_HINT_POLICY_H

#include <algorithm>
#include <cstdint>
#include <optional>
#include <vector>

namespace android {

/* What the core's PowerPort events become as power hints, without the
 * power HAL: Camera3PowerPort sends what this returns. Kept apart so
 * the mapping is tested on the host.
 *
 * The levels' clocks are the power HAL's (device tree, the camera's
 * measured table); here only which level a load is. */
class PowerHintPolicy {
public:
    enum class Hint { Launch, Shot, StreamingLow, StreamingMid, StreamingHigh };

    struct Action {
        enum class Kind { Boost, ModeOn, ModeOff };
        Kind    kind;
        Hint    hint;
        int32_t durationMs = 0;  /* a Boost's */

        bool operator==(const Action &) const = default;
    };

    /* The streaming levels by sensor pixel rate, the boundaries between
     * the formats they are named for: 720p30 is 27.6 Mpx/s, 1080p30
     * 62.2. A mode a little over its nominal rate stays in its level;
     * 720p60 (55.3) takes the 1080p30 level, as many pixels. Nothing
     * mocha's sensors stream today reaches the high level. */
    static constexpr uint64_t kLowMaxPixelsPerSecond = 40000000;
    static constexpr uint64_t kMidMaxPixelsPerSecond = 100000000;

    /* The burst from open to the first frames: 0.34 s measured. */
    static constexpr int32_t kLaunchMs = 500;

    /* A still's burst, scaled by its pixels from what a 1080p one takes
     * from its request to its JPEG (300 ms measured), never shorter, and
     * capped where the power HAL caps it. */
    static constexpr uint64_t kShotRefPixels = 1920 * 1080;
    static constexpr int32_t  kShotRefMs     = 300;
    static constexpr int32_t  kShotMaxMs     = 1000;

    static Hint levelFor(uint64_t pixelsPerSecond) {
        if (pixelsPerSecond <= kLowMaxPixelsPerSecond) return Hint::StreamingLow;
        if (pixelsPerSecond <= kMidMaxPixelsPerSecond) return Hint::StreamingMid;
        return Hint::StreamingHigh;
    }

    static int32_t shotMsFor(uint64_t pixels) {
        const uint64_t ms = (uint64_t)kShotRefMs * pixels / kShotRefPixels;
        return (int32_t)std::clamp<uint64_t>(ms, kShotRefMs, kShotMaxMs);
    }

    std::vector<Action> opening() {
        return {{Action::Kind::Boost, Hint::Launch, kLaunchMs}};
    }

    /* A level change takes the old one off first; the same level again
     * sends nothing. */
    std::vector<Action> streaming(uint64_t pixelsPerSecond) {
        const Hint level = levelFor(pixelsPerSecond);
        if (mLevel == level) return {};
        std::vector<Action> out;
        if (mLevel) out.push_back({Action::Kind::ModeOff, *mLevel});
        out.push_back({Action::Kind::ModeOn, level});
        mLevel = level;
        return out;
    }

    std::vector<Action> stillCapture(uint64_t pixels) {
        return {{Action::Kind::Boost, Hint::Shot, shotMsFor(pixels)}};
    }

    std::vector<Action> stopped() {
        if (!mLevel) return {};
        const Hint level = *mLevel;
        mLevel.reset();
        return {{Action::Kind::ModeOff, level}};
    }

    /* The level on, for dumpsys. */
    std::optional<Hint> level() const { return mLevel; }

private:
    std::optional<Hint> mLevel;
};

} /* namespace android */

#endif /* HAL_POWER_HINT_POLICY_H */
