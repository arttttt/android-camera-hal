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

/* Host test for chooseCaptureSize, on the modes both mocha sensors
 * report (1920x1080 and 1280x720), each listed in every format as the
 * capability probe lists them. */

#include <cstdio>
#include <optional>
#include <vector>

#include "CaptureSize.h"

using namespace mocha::camera;

namespace {

int failures = 0;

#define CHECK(cond)                                                \
    do {                                                           \
        if (!(cond)) {                                             \
            std::printf("%s:%d: %s\n", __FILE__, __LINE__, #cond); \
            failures++;                                            \
        }                                                          \
    } while (0)

constexpr Size k1080{1920, 1080};
constexpr Size k720{1280, 720};

Capability mochaCapability() {
    Capability cap;
    for (Size mode : {k1080, k720})
        for (PixelFormat f : {PixelFormat::Rgba8888, PixelFormat::Ycbcr420, PixelFormat::Blob}) {
            Capability::StreamConfig s;
            s.format = f;
            s.size   = mode;
            cap.streams.push_back(s);
        }
    return cap;
}

StreamSpec out(PixelFormat format, uint32_t width, uint32_t height) {
    StreamSpec s;
    s.format = format;
    s.width  = width;
    s.height = height;
    return s;
}

bool is(const std::optional<Size> &got, Size want) {
    return got && got->width == want.width && got->height == want.height;
}

const Capability cap = mochaCapability();

/* What crashed the camera on 21.10.2026: Open Camera, after a recording
 * stops, configures one RGBA output of 320x240 -- no sensor mode. The
 * running 1080p feeds it and stays; from cold, the smallest mode that
 * feeds it. */
void testSmallOutputKeepsTheRunningMode() {
    const std::vector<StreamSpec> outs{out(PixelFormat::Rgba8888, 320, 240)};
    CHECK(is(chooseCaptureSize(cap, outs, k1080), k1080));
    CHECK(is(chooseCaptureSize(cap, outs, std::nullopt), k720));
}

/* The running mode is kept only while it feeds every output. */
void testRunningModeTooSmallIsLeft() {
    const std::vector<StreamSpec> outs{out(PixelFormat::Rgba8888, 1280, 720),
                                       out(PixelFormat::Blob, 1920, 1080)};
    CHECK(is(chooseCaptureSize(cap, outs, k720), k1080));
    CHECK(is(chooseCaptureSize(cap, outs, std::nullopt), k1080));
}

/* A YUV output is the capture: it names the mode, over the running one
 * too, and is refused at a size no mode has or when another output is
 * larger than it. */
void testYuvNamesTheMode() {
    const std::vector<StreamSpec> video{out(PixelFormat::Rgba8888, 1280, 720),
                                        out(PixelFormat::Ycbcr420, 1280, 720),
                                        out(PixelFormat::Blob, 1280, 720)};
    CHECK(is(chooseCaptureSize(cap, video, k1080), k720));

    const std::vector<StreamSpec> odd{out(PixelFormat::Ycbcr420, 640, 480)};
    CHECK(!chooseCaptureSize(cap, odd, k1080));

    const std::vector<StreamSpec> larger{out(PixelFormat::Ycbcr420, 1280, 720),
                                         out(PixelFormat::Blob, 1920, 1080)};
    CHECK(!chooseCaptureSize(cap, larger, std::nullopt));

    const std::vector<StreamSpec> two{out(PixelFormat::Ycbcr420, 1920, 1080),
                                      out(PixelFormat::Ycbcr420, 1280, 720)};
    CHECK(!chooseCaptureSize(cap, two, std::nullopt));
}

/* Where the YUV encoder scales, a YUV output is just another output. */
void testScalingYuvIsAnOutput() {
    Capability scaling = cap;
    scaling.yuvAtCaptureSizeOnly = false;
    const std::vector<StreamSpec> outs{out(PixelFormat::Ycbcr420, 640, 480)};
    CHECK(is(chooseCaptureSize(scaling, outs, std::nullopt), k720));
}

/* An output larger than any mode has nothing to feed it. */
void testNoModeFeeds() {
    const std::vector<StreamSpec> outs{out(PixelFormat::Blob, 3264, 2448)};
    CHECK(!chooseCaptureSize(cap, outs, k1080));
}

/* Covering is per side: a 4:3 output wider than tall still needs a mode
 * at least as tall as it. */
void testCoverIsPerSide() {
    const std::vector<StreamSpec> outs{out(PixelFormat::Rgba8888, 1024, 768)};
    CHECK(is(chooseCaptureSize(cap, outs, std::nullopt), k1080));
}

} // namespace

int main() {
    testSmallOutputKeepsTheRunningMode();
    testRunningModeTooSmallIsLeft();
    testYuvNamesTheMode();
    testScalingYuvIsAnOutput();
    testNoModeFeeds();
    testCoverIsPerSide();
    if (failures == 0)
        std::printf("capture_size_test: all passed\n");
    return failures == 0 ? 0 : 1;
}
