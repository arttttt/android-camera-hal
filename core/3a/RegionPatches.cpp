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

#include "RegionPatches.h"

#include "sensor/SensorGeometry.h"

namespace android {

using mocha::camera::MeteringRegion;
using mocha::camera::Rect;
using mocha::camera::SensorRect;
using mocha::camera::Size;

namespace {

/* Expand [lo, hi) to at least `minSpan` patches, growing around its
 * centre. Clamped to [0, gridLimit]; if the expansion would push past
 * an edge, slide the rectangle inward so the full minSpan still fits
 * when there's room (only narrows below minSpan when the grid itself
 * is smaller, which can't happen for the IpaStats 16×16 grid +
 * minSpan ≤ 16). */
void growToMin(int *lo, int *hi, int gridLimit, int minSpan) {
    int span = *hi - *lo;
    if (span >= minSpan) return;
    const int centre = (*lo + *hi) / 2;
    int newLo = centre - minSpan / 2;
    int newHi = newLo + minSpan;
    if (newLo < 0) {
        newHi -= newLo;
        newLo = 0;
    }
    if (newHi > gridLimit) {
        newLo -= newHi - gridLimit;
        newHi  = gridLimit;
        if (newLo < 0) newLo = 0;
    }
    *lo = newLo;
    *hi = newHi;
}

} /* namespace */

std::optional<PatchRoi> regionPatches(const FrameMap &map, const MeteringRegion &region,
                                      int minSpan) {
    if (region.weight <= 0 || !map.known())
        return std::nullopt;

    const SensorRect &r = region.rect;
    const Rect visible = map.visible();
    /* Apps routinely send "no specific subject" as a near-full-frame
     * rectangle (Camera2's `Camera.Region` weight doesn't map cleanly
     * onto our patch grid, so the convention is to either set zero
     * regions or hand back almost the whole frame). The exact-full
     * check misses the off-by-one variants apps use -- (width - 1) is
     * common -- and would parse them as a real region spanning the
     * whole grid. Anything within one pixel of the edges of what the
     * frame shows is full-frame. */
    const bool isFullFrame =
        (r.xMin <= visible.left + 1 && r.yMin <= visible.top + 1
         && r.xMax >= visible.left + visible.width - 1
         && r.yMax >= visible.top + visible.height - 1);
    if (isFullFrame)
        return std::nullopt;

    /* Active-array coordinates → the frame's pixels, through the
     * mode's window. */
    const Rect f = map.toFrame(map.clip(Rect{r.xMin, r.yMin, r.xMax - r.xMin, r.yMax - r.yMin}));
    if (f.empty())
        return std::nullopt;

    /* Frame pixels → patch grid. Floor on Lo, ceil on Hi so a tiny
     * region still spans at least one patch column / row. */
    const Size out = map.output();
    const float wPerPatch = (float)out.width  / IpaStats::PATCH_X;
    const float hPerPatch = (float)out.height / IpaStats::PATCH_Y;
    int pxLo = (int)((float)f.left / wPerPatch);
    int pyLo = (int)((float)f.top / hPerPatch);
    int pxHi = (int)((float)(f.left + f.width) / wPerPatch + 0.999f);
    int pyHi = (int)((float)(f.top + f.height) / hPerPatch + 0.999f);
    if (pxLo < 0) pxLo = 0;
    if (pyLo < 0) pyLo = 0;
    if (pxHi > IpaStats::PATCH_X) pxHi = IpaStats::PATCH_X;
    if (pyHi > IpaStats::PATCH_Y) pyHi = IpaStats::PATCH_Y;
    if (pxHi <= pxLo || pyHi <= pyLo)
        return std::nullopt;

    growToMin(&pxLo, &pxHi, IpaStats::PATCH_X, minSpan);
    growToMin(&pyLo, &pyHi, IpaStats::PATCH_Y, minSpan);
    return PatchRoi{pyLo, pyHi, pxLo, pxHi};
}

} /* namespace android */
