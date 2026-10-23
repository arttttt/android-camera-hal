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

#ifndef MOCHACAMERA_FRAME_SETTINGS_H
#define MOCHACAMERA_FRAME_SETTINGS_H

#include <cstdint>
#include <optional>
#include <string>

#include "mochacamera/Controls.h"

namespace mocha::camera {

/* What one capture request asks of the core, parsed once by the
 * frontend when the request is accepted. Every field the core reads is
 * here; nothing in the core looks at the frontend's metadata.
 *
 * A field that is optional is one the request may leave out, and its
 * absence means something of its own (keep the default exposure, no
 * thumbnail, centre metering); the rest carry the value Camera2 gives
 * an absent key. */
struct FrameSettings {
    struct Ae {
        AeMode  mode = AeMode::On;
        bool    lock = false;
        /* In steps of Capability::ae.evStep. */
        int32_t evCompensationSteps = 0;
        /* Where to meter; without one, AE meters where AF looks. */
        std::optional<MeteringRegion> region;
    } ae;

    struct Awb {
        AwbMode mode = AwbMode::Auto;
        bool    lock = false;
        /* COLOR_CORRECTION_GAINS for AWB Off, as R, G (the two greens
         * averaged: they are calibrated equal) and B. Absent in any
         * other mode, so the auto gains are never frozen by the gains
         * the framework echoes back. */
        struct Gains { float r = 1.f; float g = 1.f; float b = 1.f; };
        std::optional<Gains> manualGains;
    } awb;

    struct Af {
        AfMode    mode    = AfMode::Off;
        AfTrigger trigger = AfTrigger::Idle;
        std::optional<MeteringRegion> region;
        /* LENS_FOCUS_DISTANCE in diopters, for AF Off only. */
        std::optional<float> focusDistanceDiopters;
    } af;

    /* The sensor values a manual request names. */
    struct Sensor {
        std::optional<int64_t> exposureTimeNs;
        std::optional<int32_t> sensitivityIso;
    } sensor;

    std::optional<CropRegion> crop;

    CaptureIntent intent = CaptureIntent::Preview;

    struct Jpeg {
        std::optional<uint8_t> quality;
        int32_t  orientationDegrees = 0;
        /* 0 x 0: no thumbnail. */
        uint32_t thumbnailWidth   = 0;
        uint32_t thumbnailHeight  = 0;
        std::optional<uint8_t> thumbnailQuality;
        /* JPEG_GPS_COORDINATES, _TIMESTAMP and _PROCESSING_METHOD:
         * where the picture was taken, for its EXIF; absent when the
         * request gives no coordinates. */
        struct Gps {
            double      latitude  = 0;
            double      longitude = 0;
            double      altitude  = 0;
            int64_t     timestampSec = 0;
            std::string processingMethod;
        };
        std::optional<Gps> gps;
    } jpeg;

    /* A snapshot -- a still capture, a video snapshot or a ZSL request,
     * or any request with a BLOB output -- is never given up by the AE
     * mode switch. */
    bool stillCapture = false;
};

} /* namespace mocha::camera */

#endif /* MOCHACAMERA_FRAME_SETTINGS_H */
