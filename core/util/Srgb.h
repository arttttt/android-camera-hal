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

#ifndef CORE_UTIL_SRGB_H
#define CORE_UTIL_SRGB_H

#include <math.h>

namespace android {

/* The piecewise sRGB transfer (IEC 61966-2-1), both ways, on [0, 1].
 * The gamma LUT encodes with srgbEncode; whatever reasons about the
 * rendered value from the linear side decodes with srgbDecode, so the
 * two stay one curve. */
inline float srgbEncode(float lin) {
    return (lin <= 0.0031308f) ? lin * 12.92f
                               : 1.055f * powf(lin, 1.0f / 2.4f) - 0.055f;
}

inline float srgbDecode(float enc) {
    return (enc <= 0.04045f) ? enc / 12.92f
                             : powf((enc + 0.055f) / 1.055f, 2.4f);
}

}  // namespace android

#endif  // CORE_UTIL_SRGB_H
