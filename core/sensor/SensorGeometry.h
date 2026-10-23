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

#ifndef SENSOR_GEOMETRY_H
#define SENSOR_GEOMETRY_H

#include <optional>
#include <vector>

#include "mochacamera/Controls.h"
#include "mochacamera/FrameSettings.h"

namespace android {

/* A sensor mode: the frame it outputs, and the window of the pixel array
 * that frame shows, in pixel-array coordinates -- as the sensor's driver
 * reports it (the selection API's CROP). */
struct SensorMode {
    mocha::camera::Size output;
    mocha::camera::Rect window;
};

/* The pixel array, the modes on it, and the active array Camera2 is
 * told of: the field the modes show between them, in pixel-array
 * coordinates. Requests' regions and crops are in active-array
 * coordinates, its top left at (0, 0). */
struct SensorGeometry {
    mocha::camera::Size pixelArray;
    mocha::camera::Rect activeArray;
    std::vector<SensorMode> modes;

    /* The active array is the box around the modes' windows, inside
     * the pixel array. */
    static SensorGeometry fromModes(mocha::camera::Size pixelArray,
                                    std::vector<SensorMode> modes);

    /* The mode that outputs `output`; none when no mode does. */
    std::optional<SensorMode> modeFor(mocha::camera::Size output) const;
};

/* One mode's frame against the active array: what of the active array
 * the frame shows, and the mapping between the two -- the window's
 * origin taken off, then scaled from the window to the output (what
 * libcamera's Raspberry Pi pipeline does with a ScalerCrop). */
class FrameMap {
public:
    /* A frame that shows the whole active array 1:1. */
    FrameMap() = default;
    FrameMap(const mocha::camera::Rect &activeArray, const SensorMode &mode);

    /* False for the default map: no mode's window known. */
    bool known() const { return !mWindow.empty(); }

    mocha::camera::Size output() const { return mOutput; }

    /* The part of the active array the frame shows, in active-array
     * coordinates. */
    mocha::camera::Rect visible() const { return mVisible; }

    /* `r` within what the frame shows; empty when it is all outside. */
    mocha::camera::Rect clip(const mocha::camera::Rect &r) const;

    /* An active-array rectangle in the frame's pixels, edges rounded
     * to the nearest, kept within the frame. */
    mocha::camera::Rect toFrame(const mocha::camera::Rect &r) const;

private:
    mocha::camera::Size mOutput;
    mocha::camera::Rect mVisible;
    /* The window, in active-array coordinates. */
    mocha::camera::Rect mWindow;
};

/* What a frame uses of its request's crop and metering regions, by
 * Camera2's rules, written back into `settings` for the frame and its
 * result: the crop within what the frame shows (all of that when the
 * crop falls wholly outside); each region within the crop, dropped --
 * and so not reported -- when it falls wholly outside. Nothing changes
 * under a map that knows no window. */
void fitToFrame(mocha::camera::FrameSettings &settings, const FrameMap &map);

} /* namespace android */

#endif /* SENSOR_GEOMETRY_H */
