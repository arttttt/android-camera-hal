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

/* Host test for desaturateCcmQ10: the dark-scene chroma scale folded
 * into the CCM. On the rear's daylight matrix. */

#include "3a/CcmSaturation.h"
#include "Check.h"

#include <math.h>
#include <stdlib.h>

#include <initializer_list>

using android::desaturateCcmQ10;
using mocha::camera::test::check;
using mocha::camera::test::finish;

namespace {

/* IMX179's ~5000 K CCM, row-major Q10: rows sum to 1024. */
constexpr int16_t kCcm[9] = {
    1820, -620,  -176,
    -280, 1460,  -156,
     -40, -560,  1624,
};

/* The CCM applied to a camera-RGB triple, in Q10 output units. */
void apply(const int16_t m[9], const int rgb[3], float out[3]) {
    for (int r = 0; r < 3; ++r)
        out[r] = (float)(m[r * 3] * rgb[0] + m[r * 3 + 1] * rgb[1]
                       + m[r * 3 + 2] * rgb[2]);
}

float luma(const float v[3]) {
    return 0.2126f * v[0] + 0.7152f * v[1] + 0.0722f * v[2];
}

/* s = 1 leaves the matrix as it was, to the code. */
void testFullSaturationIsTheCcm() {
    int16_t out[9];
    desaturateCcmQ10(kCcm, 1.f, out);
    for (int i = 0; i < 9; ++i) check(out[i] == kCcm[i]);
}

/* s = 0 renders every input grey: the three outputs agree. */
void testZeroSaturationIsGrey() {
    int16_t out[9];
    desaturateCcmQ10(kCcm, 0.f, out);
    const int bluish[3] = {200, 400, 700};
    float v[3];
    apply(out, bluish, v);
    check(v[0] == v[1] && v[1] == v[2]);
}

/* Luminance is kept at any s: only the colour goes. The tolerance is
 * the Q10 rounding, half a code on each of three coefficients. */
void testLuminanceIsKept() {
    const int colours[3][3] = {{200, 400, 700}, {900, 300, 100}, {512, 512, 512}};
    for (float s : {0.f, 0.25f, 0.6f, 1.f}) {
        int16_t out[9];
        desaturateCcmQ10(kCcm, s, out);
        for (const auto &rgb : colours) {
            float a[3], b[3];
            apply(kCcm, rgb, a);
            apply(out, rgb, b);
            check(fabsf(luma(a) - luma(b)) <= 1.5f * 1024.f);
        }
    }
}

/* White stays white: rows that sum to unity still do. */
void testWhiteStaysWhite() {
    for (float s : {0.f, 0.25f, 0.6f}) {
        int16_t out[9];
        desaturateCcmQ10(kCcm, s, out);
        for (int r = 0; r < 3; ++r)
            check(abs(out[r * 3] + out[r * 3 + 1] + out[r * 3 + 2] - 1024) <= 2);
    }
}

/* Out-of-range s clamps: above 1 is the CCM, below 0 is grey. */
void testSaturationClamps() {
    int16_t over[9], one[9], under[9], zero[9];
    desaturateCcmQ10(kCcm, 1.5f, over);
    desaturateCcmQ10(kCcm, 1.f, one);
    desaturateCcmQ10(kCcm, -0.5f, under);
    desaturateCcmQ10(kCcm, 0.f, zero);
    for (int i = 0; i < 9; ++i) {
        check(over[i] == one[i]);
        check(under[i] == zero[i]);
    }
}

} // namespace

int main() {
    testFullSaturationIsTheCcm();
    testZeroSaturationIsGrey();
    testLuminanceIsKept();
    testWhiteStaysWhite();
    testSaturationClamps();
    return finish("ccm_saturation_test");
}
