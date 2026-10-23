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

/* Host test for regionPatches: the stats patches a metering region
 * covers on a mode's frame -- the one path AF and AE regions take. On
 * the windows mocha's IMX179 driver reports. */

#include "3a/RegionPatches.h"
#include "sensor/SensorGeometry.h"
#include "Check.h"

using android::FrameMap;
using android::PatchRoi;
using android::SensorGeometry;
using android::SensorMode;
using android::regionPatches;
using mocha::camera::MeteringRegion;
using mocha::camera::Rect;
using mocha::camera::Size;
using mocha::camera::test::check;
using mocha::camera::test::finish;

namespace {

SensorGeometry imx179() {
    return SensorGeometry::fromModes(Size{3280, 2464}, {
        SensorMode{Size{1920, 1080}, Rect{20, 320, 3240, 1823}},
        SensorMode{Size{1280, 720},  Rect{360, 512, 2560, 1440}},
    });
}

FrameMap frameOf(Size output) {
    const SensorGeometry g = imx179();
    return FrameMap(g.activeArray, *g.modeFor(output));
}

MeteringRegion region(int32_t xMin, int32_t yMin, int32_t xMax, int32_t yMax,
                      int32_t weight = 1000) {
    MeteringRegion r;
    r.rect.xMin = xMin;
    r.rect.yMin = yMin;
    r.rect.xMax = xMax;
    r.rect.yMax = yMax;
    r.weight    = weight;
    return r;
}

bool is(const std::optional<PatchRoi> &r, int pyLo, int pyHi, int pxLo, int pxHi) {
    return r && r->pyLo == pyLo && r->pyHi == pyHi && r->pxLo == pxLo && r->pxHi == pxHi;
}

/* A tap in the corner: whole patches around it, grown to the minimum
 * and slid back inside the grid. */
void testTapCoversItsPatches() {
    const FrameMap m = frameOf(Size{1920, 1080});
    const MeteringRegion tap = region(0, 0, 300, 200);
    check(is(regionPatches(m, tap, 1), 0, 2, 0, 2));
    check(is(regionPatches(m, tap, 5), 0, 5, 0, 5));
}

/* Regions that take no part: weight 0, the whole frame (and the
 * off-by-one whole frame apps send), one outside what the frame
 * shows, any under a map that knows no window. */
void testRegionsThatTakeNoPart() {
    const FrameMap m1080 = frameOf(Size{1920, 1080});
    check(!regionPatches(m1080, region(0, 0, 300, 200, 0), 1));
    check(!regionPatches(m1080, region(0, 0, 3240, 1823), 1));
    check(!regionPatches(m1080, region(1, 1, 3239, 1822), 1));
    check(!regionPatches(frameOf(Size{1280, 720}), region(0, 0, 100, 100), 1));
    check(!regionPatches(FrameMap(), region(0, 0, 300, 200), 1));
}

/* On 720p the middle of the active array is the middle of the frame:
 * the window's origin comes off before the grid. */
void testNarrowModeMiddleIsTheGridsMiddle() {
    const FrameMap m = frameOf(Size{1280, 720});
    const MeteringRegion middle = region(1556, 876, 1684, 948);
    check(is(regionPatches(m, middle, 1), 7, 9, 7, 9));
}

} // namespace

int main() {
    testTapCoversItsPatches();
    testRegionsThatTakeNoPart();
    testNarrowModeMiddleIsTheGridsMiddle();
    return finish("region_patches_test");
}
