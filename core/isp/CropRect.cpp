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

#include "CropRect.h"

#include <cstdint>

namespace android {

CropRect cropForOutput(const CropRect &frame, unsigned width, unsigned height) {
    if (width == 0 || height == 0 || frame.w <= 0 || frame.h <= 0)
        return frame;

    /* Whichever side of the crop is relatively longer than the output's
     * gives way. */
    int64_t w = frame.w;
    int64_t h = frame.h;
    if (w * height > h * width)
        w = h * width / height;
    else
        h = w * height / width;
    if (w < 1) w = 1;
    if (h < 1) h = 1;

    CropRect out;
    out.x = frame.x + (int)((frame.w - w) / 2);
    out.y = frame.y + (int)((frame.h - h) / 2);
    out.w = (int)w;
    out.h = (int)h;
    return out;
}

} /* namespace android */
