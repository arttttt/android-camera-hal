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

#include "SensorSequence.h"

namespace android {

uint32_t SensorSequence::next(uint32_t driverSeq, int64_t timestampNs,
                              const PeriodOf &periodOf) {
    if (!started) {
        started    = true;
        lastDriver = driverSeq;
        lastNs     = timestampNs;
        lastSeq    = 0;
        missed     = 0;
        return 0;
    }

    /* At least the frames the driver counted; a driver count that did
     * not move (it never should) is one frame. */
    uint32_t counted = driverSeq - lastDriver;
    if (counted == 0) counted = 1;

    const int64_t elapsed = timestampNs - lastNs;
    uint32_t frames = 0;
    int64_t  spanned = 0;
    for (; frames < counted; ++frames)
        spanned += periodOf(lastSeq + frames + 1);

    /* Frames the driver missed: while what is left of the interval
     * holds at least half of the next frame. A period the record cannot
     * tell (0) ends the search -- the driver's count is all there is.
     * Each step spans a whole frame, so the search ends within the
     * interval's frames. */
    for (;;) {
        const int64_t period = periodOf(lastSeq + frames + 1);
        if (period <= 0 || elapsed - spanned < period / 2)
            break;
        spanned += period;
        ++frames;
        ++missed;
    }

    lastDriver = driverSeq;
    lastNs     = timestampNs;
    lastSeq   += frames;
    return lastSeq;
}

} /* namespace android */
