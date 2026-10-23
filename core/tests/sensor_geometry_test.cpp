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

/* Host test for the sensor geometry: the active array from the modes'
 * windows, and a mode's frame against it -- on the windows mocha's
 * IMX179 driver reports. */

#include "sensor/SensorGeometry.h"
#include "Check.h"

using android::FrameMap;
using android::SensorGeometry;
using android::SensorMode;
using android::fitToFrame;
using mocha::camera::FrameSettings;
using mocha::camera::MeteringRegion;
using mocha::camera::Rect;
using mocha::camera::Size;
using mocha::camera::test::check;
using mocha::camera::test::finish;

namespace {

bool is(const Rect &r, int32_t left, int32_t top, int32_t width, int32_t height) {
    return r.left == left && r.top == top && r.width == width && r.height == height;
}

/* IMX179 on mocha: 1080p scales a 3240-wide window, 720p bins a
 * narrower one, both centred in the 3280x2464 array. */
SensorGeometry imx179() {
    return SensorGeometry::fromModes(Size{3280, 2464}, {
        SensorMode{Size{1920, 1080}, Rect{20, 320, 3240, 1823}},
        SensorMode{Size{1280, 720},  Rect{360, 512, 2560, 1440}},
    });
}

FrameMap frameOf(const SensorGeometry &g, Size output) {
    return FrameMap(g.activeArray, *g.modeFor(output));
}

/* The active array is the field the modes show between them: here the
 * widest mode's window. */
void testActiveArrayIsTheModesField() {
    const SensorGeometry g = imx179();
    check(is(g.activeArray, 20, 320, 3240, 1823));
    check(g.modeFor(Size{1280, 720}).has_value());
    check(!g.modeFor(Size{640, 480}).has_value());
}

/* A window past the array's edge counts for what is on it. */
void testWindowsKeptToTheArray() {
    const SensorGeometry g = SensorGeometry::fromModes(Size{100, 100}, {
        SensorMode{Size{50, 50}, Rect{-10, 20, 60, 200}},
    });
    check(is(g.activeArray, 0, 20, 50, 80));
}

/* No window reported: the active array is the whole pixel array. */
void testNoWindowsIsTheWholeArray() {
    const SensorGeometry g = SensorGeometry::fromModes(Size{1920, 1080}, {});
    check(is(g.activeArray, 0, 0, 1920, 1080));
}

/* 1080p shows the whole active array, scaled into the frame. */
void testWidestModeShowsTheWholeActiveArray() {
    const FrameMap m = frameOf(imx179(), Size{1920, 1080});
    check(is(m.visible(), 0, 0, 3240, 1823));
    check(is(m.toFrame(Rect{0, 0, 3240, 1823}), 0, 0, 1920, 1080));
    check(is(m.toFrame(Rect{1620, 0, 1620, 1823}), 960, 0, 960, 1080));
}

/* 720p shows the middle of the active array: its origin comes off
 * before the scale, so the middle maps to the middle -- not to where a
 * plain scale of the active array would put it. */
void testNarrowModeTakesItsWindowsOrigin() {
    const FrameMap m = frameOf(imx179(), Size{1280, 720});
    check(is(m.visible(), 340, 192, 2560, 1440));
    check(is(m.toFrame(m.visible()), 0, 0, 1280, 720));
    check(is(m.toFrame(Rect{340 + 256, 192 + 144, 512, 288}), 128, 72, 256, 144));
    /* Beyond the window: kept to the frame. */
    check(is(m.toFrame(Rect{0, 0, 3240, 1823}), 0, 0, 1280, 720));
}

void testClip() {
    const FrameMap m = frameOf(imx179(), Size{1280, 720});
    check(is(m.clip(Rect{0, 0, 500, 300}), 340, 192, 160, 108));
    check(m.clip(Rect{0, 0, 100, 100}).empty());
}

/* The crop is what the frame shows of it; the AF region what the crop
 * holds of it, as both the frame and its result use them. */
void testFitToFrame() {
    const FrameMap m = frameOf(imx179(), Size{1280, 720});

    FrameSettings s;
    s.crop = Rect{0, 0, 3240, 1823};
    MeteringRegion tap;
    tap.rect.xMin = 0;
    tap.rect.yMin = 0;
    tap.rect.xMax = 500;
    tap.rect.yMax = 300;
    tap.weight    = 1000;
    s.af.region = tap;
    fitToFrame(s, m);
    check(is(*s.crop, 340, 192, 2560, 1440));
    check(s.af.region.has_value());
    check(s.af.region->rect.xMin == 340 && s.af.region->rect.yMin == 192);
    check(s.af.region->rect.xMax == 500 && s.af.region->rect.yMax == 300);
    check(s.af.region->weight == 1000);

    /* A region wholly outside the crop is not used, nor reported. */
    FrameSettings zoomed;
    zoomed.crop = Rect{1000, 600, 1000, 600};
    zoomed.af.region = tap;
    fitToFrame(zoomed, m);
    check(is(*zoomed.crop, 1000, 600, 1000, 600));
    check(!zoomed.af.region.has_value());

    /* A crop wholly outside the frame: all the frame shows. */
    FrameSettings outside;
    outside.crop = Rect{0, 0, 100, 100};
    fitToFrame(outside, m);
    check(is(*outside.crop, 340, 192, 2560, 1440));

    /* No crop asked: none made up; the region kept to what shows. */
    FrameSettings bare;
    bare.af.region = tap;
    fitToFrame(bare, m);
    check(!bare.crop.has_value());
    check(bare.af.region->rect.xMin == 340);
}

/* A map that knows no window changes nothing. */
void testUnknownMapLeavesSettings() {
    FrameSettings s;
    s.crop = Rect{0, 0, 10, 10};
    fitToFrame(s, FrameMap());
    check(is(*s.crop, 0, 0, 10, 10));
}

} // namespace

int main() {
    testActiveArrayIsTheModesField();
    testWindowsKeptToTheArray();
    testNoWindowsIsTheWholeArray();
    testWidestModeShowsTheWholeActiveArray();
    testNarrowModeTakesItsWindowsOrigin();
    testClip();
    testFitToFrame();
    testUnknownMapLeavesSettings();
    return finish("sensor_geometry_test");
}
