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

#define LOG_TAG "Cam-CaptureSize"

#include "CaptureSize.h"

#include <utils/Log.h>

namespace mocha::camera {

namespace {

bool feeds(Size mode, uint32_t width, uint32_t height) {
    return mode.width >= width && mode.height >= height;
}

bool isMode(const Capability &cap, Size size) {
    for (const Capability::StreamConfig &s : cap.streams)
        if (s.size.width == size.width && s.size.height == size.height)
            return true;
    return false;
}

} /* namespace */

std::optional<Size> chooseCaptureSize(const Capability &cap,
                                      std::span<const StreamSpec> outputs,
                                      std::optional<Size> running) {
    uint32_t width  = 0;
    uint32_t height = 0;
    std::optional<Size> yuv;
    for (const StreamSpec &o : outputs) {
        if (o.width  > width)  width  = o.width;
        if (o.height > height) height = o.height;
        if (o.format != PixelFormat::Ycbcr420 || !cap.yuvAtCaptureSizeOnly)
            continue;
        const Size size{o.width, o.height};
        if (yuv && (yuv->width != size.width || yuv->height != size.height)) {
            ALOGE("YUV outputs at %ux%u and %ux%u: one capture feeds one size",
                  yuv->width, yuv->height, size.width, size.height);
            return std::nullopt;
        }
        yuv = size;
    }

    if (yuv) {
        if (!isMode(cap, *yuv)) {
            ALOGE("YUV output %ux%u is no sensor mode", yuv->width, yuv->height);
            return std::nullopt;
        }
        if (!feeds(*yuv, width, height)) {
            ALOGE("YUV output %ux%u is the capture, smaller than an output of %ux%u",
                  yuv->width, yuv->height, width, height);
            return std::nullopt;
        }
        return yuv;
    }

    if (running && feeds(*running, width, height))
        return running;

    std::optional<Size> best;
    for (const Capability::StreamConfig &s : cap.streams) {
        if (!feeds(s.size, width, height))
            continue;
        if (!best || (uint64_t)s.size.width * s.size.height <
                     (uint64_t)best->width * best->height)
            best = s.size;
    }
    if (!best)
        ALOGE("no sensor mode feeds an output of %ux%u", width, height);
    return best;
}

} /* namespace mocha::camera */
