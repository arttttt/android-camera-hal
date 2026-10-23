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

#include "SensorGeometry.h"

#include <algorithm>
#include <stdint.h>

namespace android {

using mocha::camera::Rect;
using mocha::camera::Size;

namespace {

Rect intersect(const Rect &a, const Rect &b) {
    const int32_t left   = std::max(a.left, b.left);
    const int32_t top    = std::max(a.top, b.top);
    const int32_t right  = std::min(a.left + a.width, b.left + b.width);
    const int32_t bottom = std::min(a.top + a.height, b.top + b.height);
    if (right <= left || bottom <= top) return Rect{};
    return Rect{left, top, right - left, bottom - top};
}

/* `v` scaled by num / den, rounded to the nearest. */
int32_t scale(int64_t v, int64_t num, int64_t den) {
    const int64_t p = v * num;
    return (int32_t)(p >= 0 ? (p + den / 2) / den : -((-p + den / 2) / den));
}

} /* namespace */

SensorGeometry SensorGeometry::fromModes(Size pixelArray,
                                         std::vector<SensorMode> modes) {
    const Rect array{0, 0, (int32_t)pixelArray.width, (int32_t)pixelArray.height};
    Rect box;
    for (const SensorMode &m : modes) {
        const Rect w = intersect(m.window, array);
        if (w.empty()) continue;
        if (box.empty()) {
            box = w;
            continue;
        }
        const int32_t right  = std::max(box.left + box.width, w.left + w.width);
        const int32_t bottom = std::max(box.top + box.height, w.top + w.height);
        box.left   = std::min(box.left, w.left);
        box.top    = std::min(box.top, w.top);
        box.width  = right - box.left;
        box.height = bottom - box.top;
    }

    SensorGeometry g;
    g.pixelArray  = pixelArray;
    g.activeArray = box.empty() ? array : box;
    g.modes       = std::move(modes);
    return g;
}

std::optional<SensorMode> SensorGeometry::modeFor(Size output) const {
    for (const SensorMode &m : modes)
        if (m.output.width == output.width && m.output.height == output.height)
            return m;
    return std::nullopt;
}

FrameMap::FrameMap(const Rect &activeArray, const SensorMode &mode)
    : mOutput(mode.output) {
    mWindow = Rect{mode.window.left - activeArray.left,
                   mode.window.top - activeArray.top,
                   mode.window.width, mode.window.height};
    mVisible = intersect(mWindow, Rect{0, 0, activeArray.width, activeArray.height});
}

Rect FrameMap::clip(const Rect &r) const {
    if (mWindow.empty()) return r;
    return intersect(r, mVisible);
}

Rect FrameMap::toFrame(const Rect &r) const {
    if (mWindow.empty()) return r;
    const int32_t outW = (int32_t)mOutput.width;
    const int32_t outH = (int32_t)mOutput.height;
    const int32_t x0 = std::clamp(scale(r.left - mWindow.left, outW, mWindow.width), 0, outW);
    const int32_t y0 = std::clamp(scale(r.top - mWindow.top, outH, mWindow.height), 0, outH);
    const int32_t x1 = std::clamp(scale(r.left + r.width - mWindow.left, outW, mWindow.width),
                                  0, outW);
    const int32_t y1 = std::clamp(scale(r.top + r.height - mWindow.top, outH, mWindow.height),
                                  0, outH);
    return Rect{x0, y0, x1 - x0, y1 - y0};
}

namespace {

/* `region` within `crop`; dropped when it falls wholly outside. */
void fitRegion(std::optional<mocha::camera::MeteringRegion> &region, const Rect &crop) {
    if (!region) return;
    mocha::camera::SensorRect &r = region->rect;
    const Rect in = intersect(Rect{r.xMin, r.yMin, r.xMax - r.xMin, r.yMax - r.yMin}, crop);
    if (in.empty()) {
        region.reset();
        return;
    }
    r.xMin = in.left;
    r.yMin = in.top;
    r.xMax = in.left + in.width;
    r.yMax = in.top + in.height;
}

} /* namespace */

void fitToFrame(mocha::camera::FrameSettings &settings, const FrameMap &map) {
    if (!map.known()) return;

    Rect crop = map.visible();
    if (settings.crop) {
        const Rect c = map.clip(*settings.crop);
        if (!c.empty()) crop = c;
        settings.crop = crop;
    }

    fitRegion(settings.ae.region, crop);
    fitRegion(settings.af.region, crop);
}

} /* namespace android */
