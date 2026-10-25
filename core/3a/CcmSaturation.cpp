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

#include "CcmSaturation.h"

#include <math.h>

namespace android {

void desaturateCcmQ10(const int16_t in[9], float s, int16_t out[9]) {
    if (s > 1.f) s = 1.f;
    if (s < 0.f) s = 0.f;
    static constexpr float kLuma[3] = {0.2126f, 0.7152f, 0.0722f};
    float y[3];
    for (int c = 0; c < 3; ++c)
        y[c] = kLuma[0] * in[c] + kLuma[1] * in[3 + c] + kLuma[2] * in[6 + c];
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            out[r * 3 + c] = (int16_t)lrintf(s * in[r * 3 + c] + (1.f - s) * y[c]);
}

} /* namespace android */
