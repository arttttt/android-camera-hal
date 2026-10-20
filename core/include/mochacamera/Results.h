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

#ifndef MOCHACAMERA_RESULTS_H
#define MOCHACAMERA_RESULTS_H

#include <cstdint>

#include "mochacamera/Controls.h"

namespace mocha::camera {

/* What 3A says about one frame: the states the framework is shown, and
 * the sensor values that were in effect on it. The core derives the
 * states (the frontend no longer reads modes and locks back out of the
 * request to guess them). */
struct Core3AResult {
    AeState  aeState  = AeState::Inactive;
    AwbState awbState = AwbState::Inactive;
    AfState  afState  = AfState::Inactive;
    AfMode   afMode   = AfMode::Off;
    float    focusDistanceDiopters = 0.f;

    /* In effect on this frame -- what was written a control delay
     * before, or the defaults before the first write landed. */
    int64_t  exposureTimeNs   = 0;
    int32_t  sensitivityIso   = 0;
    int64_t  frameDurationNs  = 0;
};

/* Partial results, in the order a frame gets them: AWB from the 3A
 * tick, then AE from the same tick; the final result, which carries the
 * buffers, comes after both. */
enum class PartialResult : uint8_t {
    Awb = 1,
    Ae  = 2,
};

/* How a frame ended, for its final result. */
enum class FrameOutcome : uint8_t {
    Completed = 0,  /* buffers and metadata; an output may still fail */
    Failed    = 1,  /* the request as a whole; buffers in error */
};

struct CoreFrameResult {
    FrameOutcome outcome     = FrameOutcome::Completed;
    int64_t      timestampNs = 0;
    Core3AResult a3;
};

} /* namespace mocha::camera */

#endif /* MOCHACAMERA_RESULTS_H */
