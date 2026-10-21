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

/* Host test for cropForOutput: what each output samples of the frame's
 * crop. */

#include "CropRect.h"
#include "Check.h"

using android::CropRect;
using android::cropForOutput;
using mocha::camera::test::check;
using mocha::camera::test::finish;

namespace {

bool is(const CropRect &r, int x, int y, int w, int h) {
    return r.x == x && r.y == y && r.w == w && r.h == h;
}

/* An output of the crop's own aspect samples all of it, at any size. */
void testSameAspectIsTheWholeCrop() {
    const CropRect frame{0, 0, 1920, 1080};
    check(is(cropForOutput(frame, 1920, 1080), 0, 0, 1920, 1080));
    check(is(cropForOutput(frame, 1280, 720), 0, 0, 1920, 1080));
}

/* 4:3 from 16:9: the full height, the width cut evenly at both sides --
 * the 320x240 callback Open Camera configures after a recording. */
void testNarrowerOutputCutsTheSides() {
    const CropRect frame{0, 0, 1920, 1080};
    check(is(cropForOutput(frame, 320, 240), 240, 0, 1440, 1080));
}

/* Wider than the crop: the full width, top and bottom cut evenly. */
void testWiderOutputCutsTopAndBottom() {
    const CropRect frame{0, 0, 1440, 1080};
    check(is(cropForOutput(frame, 1920, 1080), 0, 135, 1440, 810));
}

/* Digital zoom: the output's rectangle stays centred in the crop, not
 * in the frame. */
void testCentredInTheZoomedCrop() {
    const CropRect zoom{480, 270, 960, 540};
    check(is(cropForOutput(zoom, 1280, 720), 480, 270, 960, 540));
    check(is(cropForOutput(zoom, 640, 480), 600, 270, 720, 540));
}

/* A degenerate output takes the crop as it is. */
void testZeroSizedOutput() {
    const CropRect frame{10, 20, 300, 200};
    check(is(cropForOutput(frame, 0, 480), 10, 20, 300, 200));
}

} // namespace

int main() {
    testSameAspectIsTheWholeCrop();
    testNarrowerOutputCutsTheSides();
    testWiderOutputCutsTopAndBottom();
    testCentredInTheZoomedCrop();
    testZeroSizedOutput();
    return finish("crop_test");
}
