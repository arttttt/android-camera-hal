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

#include "Frame.h"

#include <unistd.h>

#include "IspPipeline.h"

namespace mocha::camera {

Frame::~Frame() {
    for (Output &out : outputs) {
        if (jpegRing && out.jpeg.ringSlot >= 0) {
            jpegRing->releaseJpegSnapshot(out.jpeg);
            out.jpeg.ringSlot = -1;
        }
        /* Handed back with the result, which takes it; one left here
         * belongs to a frame that never went out. */
        if (out.releaseFence >= 0)
            ::close(out.releaseFence);
    }
}

} /* namespace mocha::camera */
