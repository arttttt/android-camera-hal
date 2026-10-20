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

#ifndef MOCHACAMERA_CAPABILITY_H
#define MOCHACAMERA_CAPABILITY_H

#include <cstddef>
#include <cstdint>
#include <vector>

#include "mochacamera/Buffers.h"
#include "mochacamera/Controls.h"

namespace mocha::camera {

/* What the camera can do, as the core knows it: the one source the
 * frontend's static characteristics, its request templates and its
 * stream checks read, so what is advertised is what is produced.
 * Probed once per camera from the sensor, the driver and the tuning. */
struct Capability {
    bool facingFront = false;

    struct Sensor {
        /* The pixel array, which is also the active array. */
        Size    pixelArray;
        float   physicalSizeMm[2] = {0.f, 0.f};
        int32_t orientationDegrees = 0;
        /* Camera2's colour filter arrangement values. */
        enum class Cfa : uint8_t { Rggb = 0, Grbg = 1, Gbrg = 2, Bggr = 3 };
        Cfa     cfa = Cfa::Rggb;
        /* Absent (zero) when the driver could not tell. */
        int64_t exposureMinNs = 0;
        int64_t exposureMaxNs = 0;
        int32_t sensitivityMinIso = 0;
        int32_t sensitivityMaxIso = 0;
    } sensor;

    struct Lens {
        float focalLengthMm = 0.f;
        float aperture      = 2.0f;
        /* 0 for a fixed-focus lens. */
        float minFocusDistanceDiopters = 0.f;
    } lens;

    struct FpsRange {
        int32_t min = 0;
        int32_t max = 0;
    };

    struct Ae {
        std::vector<AeMode> modes;
        /* One EV compensation step, and the range in steps. */
        Rational evStep{1, 3};
        int32_t  evMinSteps = -9;
        int32_t  evMaxSteps = 9;
        std::vector<FpsRange> targetFpsRanges;
    } ae;

    struct Awb {
        std::vector<AwbMode> modes;
    } awb;

    struct Af {
        std::vector<AfMode> modes;
        /* AF regions honoured at once; AE and AWB take none. */
        int32_t maxRegions = 0;
    } af;

    /* An output the pipeline produces, at its fastest. */
    struct StreamConfig {
        PixelFormat format = PixelFormat::Rgba8888;
        Size        size;
        int64_t     minFrameDurationNs = 0;
        /* How long a frame with this output holds the pipeline up
         * beyond its frame duration. */
        int64_t     stallDurationNs = 0;
    };
    std::vector<StreamConfig> streams;

    /* What the outputs combine to: one capture size per configuration.
     * A YUV output is the capture itself (its encoder does not scale),
     * so it must be the capture size; RGBA and JPEG outputs scale. */
    bool yuvAtCaptureSizeOnly = true;

    struct Jpeg {
        /* A BLOB buffer's size, its trailer included. */
        size_t maxSize = 0;
        /* (0, 0) first: no thumbnail. */
        std::vector<Size> thumbnailSizes;
    } jpeg;

    float   maxDigitalZoom = 4.f;

    /* Frames between a request's acceptance and its result, at most. */
    int32_t pipelineMaxDepth = 4;
    /* Outputs per request: processed ones, of which stalling (JPEG). */
    int32_t maxProcessedOutputs = 2;
    int32_t maxStallingOutputs  = 1;
};

} /* namespace mocha::camera */

#endif /* MOCHACAMERA_CAPABILITY_H */
