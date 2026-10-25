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

#ifndef THREE_A_CCM_SATURATION_H
#define THREE_A_CCM_SATURATION_H

#include <stdint.h>

namespace android {

/* The CCM with its chroma scaled by `s` around luminance: each output
 * is s·rgb + (1 − s)·Y, Y the Rec. 709 luminance of the CCM's own
 * output (the CCM lands in linear sRGB). Folded into the matrix, so
 * the shader runs it for nothing: s = 1 is the CCM as it was, s = 0
 * renders grey, and Y is the same at any s — the image only loses
 * colour, never brightness. The same fold as libcamera RPi's
 * applySaturation and rkaiq's Saturationadjust. Row-major Q10 in and
 * out; s is clamped to [0, 1]. */
void desaturateCcmQ10(const int16_t in[9], float s, int16_t out[9]);

} /* namespace android */

#endif /* THREE_A_CCM_SATURATION_H */
