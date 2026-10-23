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

#define LOG_TAG "Cam-Capability"

#include "CapabilityProbe.h"

#include <string.h>
#include <sys/user.h>

#include <algorithm>

#include <linux/videodev2.h>
#include <utils/Log.h>

#include "CaptureEngine.h"
#include "OutputResolutionCap.h"
#include "V4l2Controls.h"
#include "V4l2Device.h"
#include "sensor/SensorConfig.h"
#include "sensor/SensorGeometry.h"
#include "sensor/SensorTuning.h"

namespace mocha::camera {

namespace {

/* Conservative 30 fps when the driver reports no framerate for a mode
 * -- better for the framework than a 60 fps lie. */
constexpr int64_t kFallbackMinFrameDurationNs = 1000000000LL / 30;

/* Variable target ranges start here: AE may drop the rate this far in
 * low light. */
constexpr int32_t kFpsLowerFloor = 15;
/* Bounds the list when a sensor advertises many discrete rates. */
constexpr size_t  kFpsMaxRanges  = 16;

/* The JPEG trailer at the end of a BLOB buffer (camera3_jpeg_blob). */
constexpr size_t  kJpegTrailerSize = 8;

void probeSensor(Capability &cap, android::V4l2Device &dev,
                 const android::SensorTuning *tuning,
                 const android::SensorConfig &cfg,
                 const android::SensorGeometry &geometry) {
    cap.sensor.pixelArray  = geometry.pixelArray;
    cap.sensor.activeArray = geometry.activeArray;
    cap.sensor.physicalSizeMm[0] = geometry.pixelArray.width  * cfg.pixelSizeUm / 1000.f;
    cap.sensor.physicalSizeMm[1] = geometry.pixelArray.height * cfg.pixelSizeUm / 1000.f;

    if (tuning && tuning->isLoaded())
        cap.lens.minFocusDistanceDiopters = tuning->module().minFocusDistanceDiopters;
    else
        cap.lens.minFocusDistanceDiopters = dev.hasLens() ? 10.0f : 0.0f;
    cap.lens.focalLengthMm = cfg.focalLengthMm;
    cap.sensor.orientationDegrees = tuning ? tuning->module().sensorOrientationDegrees : 0;

    if (tuning && tuning->isLoaded()) {
        const char *p = tuning->bayerPattern();
        if      (strcmp(p, "GRBG") == 0) cap.sensor.cfa = Capability::Sensor::Cfa::Grbg;
        else if (strcmp(p, "GBRG") == 0) cap.sensor.cfa = Capability::Sensor::Cfa::Gbrg;
        else if (strcmp(p, "BGGR") == 0) cap.sensor.cfa = Capability::Sensor::Cfa::Bggr;
        else                             cap.sensor.cfa = Capability::Sensor::Cfa::Rggb;
    }

    /* Exposure and sensitivity ranges from the driver. No fallback
     * constants: a range the driver cannot give is left out, and the
     * frontend leaves the key out with it. */
    int32_t expMinUs = 0, expMaxUs = 0, expDefUs = 0;
    if (dev.queryControl(V4L2_CID_EXPOSURE, &expMinUs, &expMaxUs, &expDefUs)) {
        cap.sensor.exposureMinNs = (int64_t)expMinUs * 1000LL;
        cap.sensor.exposureMaxNs = (int64_t)expMaxUs * 1000LL;
    } else {
        ALOGE("V4L2_CID_EXPOSURE query failed: no exposure range");
    }
    int32_t flMin = 0, flMax = 0, flDef = 0, lineNs = 0;
    if (dev.queryControl(V4L2_CID_FRAME_LENGTH, &flMin, &flMax, &flDef) &&
        dev.getControl(V4L2_CID_LINE_TIME_NS, &lineNs) && lineNs > 0) {
        /* The longest frame at the mode's own line, and the longest
         * exposure it holds: the line is not stretched for exposures
         * (SensorConfig::timingForExposure). */
        cap.sensor.maxFrameDurationNs = (int64_t)flMax * lineNs;
        const int64_t longest = (int64_t)(flMax - cfg.maxCoarseDiff) * lineNs;
        if (longest > 0 && cap.sensor.exposureMaxNs > longest)
            cap.sensor.exposureMaxNs = longest;
    } else {
        ALOGE("frame length / line time query failed: no max frame duration");
    }
    int32_t gMin = 0, gMax = 0, gDef = 0;
    if (dev.queryControl(V4L2_CID_GAIN, &gMin, &gMax, &gDef)) {
        int32_t isoMin = cfg.gainToIso(gMin);
        if (isoMin < 1) isoMin = 1;
        cap.sensor.sensitivityMinIso = isoMin;
        cap.sensor.maxAnalogIso      = cfg.gainToIso(gMax);
        cap.sensor.sensitivityMaxIso = cfg.sensitivityIso(gMax, cfg.digitalGainMax);
    } else {
        ALOGE("V4L2_CID_GAIN query failed: no sensitivity range");
    }
}

/* The sizes the SW ISP sustains (OutputResolutionCap), each in every
 * format, at the driver's fastest rate for the mode. */
void probeStreams(Capability &cap, android::V4l2Device &dev) {
    const auto &all = dev.availableResolutions();
    for (size_t i = 0; i < all.size(); ++i) {
        const android::Resolution &r = all[i];
        if (!android::OutputResolutionCap::accepts((int32_t)r.width, (int32_t)r.height))
            continue;
        int64_t minDur = dev.minFrameDurationNs(r.width, r.height);
        if (minDur <= 0) minDur = kFallbackMinFrameDurationNs;
        for (PixelFormat f : {PixelFormat::Rgba8888, PixelFormat::Ycbcr420, PixelFormat::Blob}) {
            Capability::StreamConfig s;
            s.format             = f;
            s.size               = Size{r.width, r.height};
            s.minFrameDurationNs = minDur;
            /* Advertised as the frame duration itself, as it always
             * was; nothing measured says otherwise. */
            s.stallDurationNs    = minDur;
            cap.streams.push_back(s);
        }
    }
}

void probeFpsRanges(Capability &cap, android::V4l2Device &dev) {
    const auto &fpsList = dev.availableFps();
    auto add = [&cap](int32_t lo, int32_t hi) {
        if (cap.ae.targetFpsRanges.size() >= kFpsMaxRanges) return;
        cap.ae.targetFpsRanges.push_back(Capability::FpsRange{lo, hi});
    };
    /* For every rate the driver advertises: a constant range for video
     * and a variable one AE may drop the rate in. */
    if (fpsList.isEmpty()) {
        add(kFpsLowerFloor, 30);
        add(30, 30);
    } else {
        for (size_t i = 0; i < fpsList.size(); ++i) {
            const int32_t fps = fpsList[i];
            if (fps > kFpsLowerFloor) add(kFpsLowerFloor, fps);
            add(fps, fps);
        }
    }
}

/* AUTO and OFF always: every AWB honours them. Presets only with the
 * Bayesian AWB and a tuning that maps them to a CT range -- gray-world
 * has none and would silently behave as AUTO. */
void probeAwbModes(Capability &cap, const android::SensorTuning *tuning) {
    cap.awb.modes = {AwbMode::Auto, AwbMode::Off};
    if (!tuning || tuning->awbAlgorithm() != android::SensorTuning::AwbAlgorithm::Bayes ||
        !tuning->bayesParams() || tuning->bayesParams()->modes.empty())
        return;
    const auto &modes = tuning->bayesParams()->modes;
    auto have = [&modes](const char *name) {
        for (size_t i = 0; i < modes.size(); ++i)
            if (modes[i].first == name) return true;
        return false;
    };
    static const struct { const char *name; AwbMode mode; } presets[] = {
        {"INCANDESCENT",     AwbMode::Incandescent},
        {"FLUORESCENT",      AwbMode::Fluorescent},
        {"WARM_FLUORESCENT", AwbMode::WarmFluorescent},
        {"DAYLIGHT",         AwbMode::Daylight},
        {"CLOUDY_DAYLIGHT",  AwbMode::CloudyDaylight},
        {"TWILIGHT",         AwbMode::Twilight},
        {"SHADE",            AwbMode::Shade},
    };
    for (const auto &p : presets)
        if (have(p.name)) cap.awb.modes.push_back(p.mode);
}

} /* namespace */

Capability probeCapability(android::V4l2Device &dev, bool facingFront,
                           const android::SensorTuning *tuning,
                           const android::SensorConfig &cfg,
                           const android::SensorGeometry &geometry,
                           int requestsInFlight) {
    Capability cap;
    cap.facingFront = facingFront;

    probeSensor(cap, dev, tuning, cfg, geometry);
    probeStreams(cap, dev);
    probeFpsRanges(cap, dev);
    probeAwbModes(cap, tuning);

    /* AE On is the IPA's; AE Off the request's own exposure and
     * sensitivity (requestExposure's manual path). */
    cap.ae.modes = {AeMode::Off, AeMode::On};

    /* AF modes where the device has a lens to move. */
    if (!dev.hasLens()) {
        cap.af.modes      = {AfMode::Off};
        cap.af.maxRegions = 1;
    } else {
        cap.af.modes      = {AfMode::Off, AfMode::Auto, AfMode::Macro,
                             AfMode::ContinuousPicture};
        cap.af.maxRegions = 1;
    }

    /* ~1.1 byte/px is a typical JPEG; 2 bytes/px of the largest JPEG
     * output for margin, the trailer, page-aligned. */
    size_t jpegPixels = 0;
    for (const Capability::StreamConfig &s : cap.streams)
        if (s.format == PixelFormat::Blob)
            jpegPixels = std::max(jpegPixels, (size_t)s.size.width * s.size.height);
    size_t jpegMax = jpegPixels * 2 + kJpegTrailerSize;
    cap.jpeg.maxSize = (jpegMax + PAGE_SIZE - 1u) & ~(PAGE_SIZE - 1u);
    /* (0,0) for no thumbnail, then 16:9 and 4:3 -- the aspect ratios of
     * the output sizes. The encoder crops the snapshot's centre to the
     * requested aspect. */
    cap.jpeg.thumbnailSizes = {Size{0, 0}, Size{320, 180}, Size{320, 240}};

    /* Requests in flight, as configured: a stream's max_buffers. Each
     * can hold a capture slot from its frame on; more than the slots
     * would starve the VI and lose frames. */
    const int32_t slots =
        V4L2DEVICE_BUF_COUNT - CaptureEngine::kSlotsBesidesRequests;
    int32_t inFlight = std::max(requestsInFlight, 1);
    if (inFlight > slots) {
        ALOGW("%d requests in flight asked, the capture holds %d: %d",
              requestsInFlight, slots, slots);
        inFlight = slots;
    }
    cap.maxBuffersPerStream = inFlight;

    /* The frames a request takes, at most: as many as are in flight
     * before it, and no fewer than its own path -- the longest control
     * delay to its frame, the frame, the GPU's frames in flight, the
     * finish and the result. */
    int32_t delay = 0;
    for (int32_t d : cfg.controlDelay) delay = std::max(delay, d);
    cap.pipelineMaxDepth = std::max<int32_t>(
        inFlight, delay + 1 + CaptureEngine::kPermits + 1);

    return cap;
}

} /* namespace mocha::camera */
