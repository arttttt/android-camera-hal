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

#ifndef MOCHACAMERA_CONTROLS_H
#define MOCHACAMERA_CONTROLS_H

#include <cstdint>

namespace mocha::camera {

/* The 3A vocabulary of the core. The values are Camera2's, set
 * explicitly: a frontend speaking Camera2 converts with a cast and a
 * static_assert per member, and nothing here depends on its headers.
 * Only what the HAL implements has a name; a frontend maps anything
 * else onto these before the core sees it. */

enum class AeMode : uint8_t {
    Off = 0,
    On  = 1,
};

enum class AeState : uint8_t {
    Inactive  = 0,
    Searching = 1,
    Converged = 2,
    Locked    = 3,
};

enum class AwbMode : uint8_t {
    Off             = 0,
    Auto            = 1,
    Incandescent    = 2,
    Fluorescent     = 3,
    WarmFluorescent = 4,
    Daylight        = 5,
    CloudyDaylight  = 6,
    Twilight        = 7,
    Shade           = 8,
};

enum class AwbState : uint8_t {
    Inactive  = 0,
    Searching = 1,
    Converged = 2,
    Locked    = 3,
};

enum class AfMode : uint8_t {
    Off               = 0,
    Auto              = 1,
    Macro             = 2,
    ContinuousVideo   = 3,
    ContinuousPicture = 4,
    Edof              = 5,
};

enum class AfTrigger : uint8_t {
    Idle   = 0,
    Start  = 1,
    Cancel = 2,
};

enum class AfState : uint8_t {
    Inactive         = 0,
    PassiveScan      = 1,
    PassiveFocused   = 2,
    ActiveScan       = 3,
    FocusedLocked    = 4,
    NotFocusedLocked = 5,
    PassiveUnfocused = 6,
};

enum class CaptureIntent : uint8_t {
    Custom         = 0,
    Preview        = 1,
    StillCapture   = 2,
    VideoRecord    = 3,
    VideoSnapshot  = 4,
    ZeroShutterLag = 5,
    Manual         = 6,
};

/* A rectangle on the sensor's active array, in its pixels: what Camera2
 * calls a region (crop, metering), half-open on neither side -- xMax
 * and yMax are the far edges as the framework gives them. */
struct SensorRect {
    int32_t xMin = 0;
    int32_t yMin = 0;
    int32_t xMax = 0;
    int32_t yMax = 0;
};

/* A corner and a size, in pixels. */
struct Rect {
    int32_t left   = 0;
    int32_t top    = 0;
    int32_t width  = 0;
    int32_t height = 0;

    bool empty() const { return width <= 0 || height <= 0; }
};

/* The crop region, as Camera2 gives it, in active-array pixels. */
using CropRegion = Rect;

/* A metering region: a rectangle and its weight, 0..1000. A weight of
 * 0 takes no part in metering (Camera2's METERING_WEIGHT_DONT_CARE):
 * a region of weight 0 is as no region at all -- apps send it to drop
 * a tap. */
struct MeteringRegion {
    SensorRect rect;
    int32_t    weight = 0;
};

/* A width and a height, in pixels. */
struct Size {
    uint32_t width  = 0;
    uint32_t height = 0;
};

/* A frame rate range, frames per second, as Camera2's AE target range
 * gives it. */
struct FpsRange {
    int32_t min = 0;
    int32_t max = 0;
};

/* An exact fraction, for steps Camera2 states as rationals. */
struct Rational {
    int32_t numerator   = 0;
    int32_t denominator = 1;

    float value() const {
        return denominator ? (float)numerator / (float)denominator : 0.f;
    }
};

} /* namespace mocha::camera */

#endif /* MOCHACAMERA_CONTROLS_H */
