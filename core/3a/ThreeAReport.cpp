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

#include "ThreeAReport.h"

#include "sensor/DelayedControls.h"
#include "sensor/SensorConfig.h"

namespace android {

using mocha::camera::AeMode;
using mocha::camera::AeState;
using mocha::camera::AwbMode;
using mocha::camera::AwbState;
using mocha::camera::Core3AResult;
using mocha::camera::FrameSettings;

Core3AResult describe3A(const FrameSettings &settings,
                        const AutoFocusController::Report &af,
                        ThreeAConvergence converged,
                        const DelayedControls::Batch &inEffect,
                        const SensorConfig &cfg) {
    Core3AResult r;

    /* A control the frame has no record of (a frame never captured) is
     * reported at what configure wrote. */
    const DelayedControls::Batch defaults = cfg.configureDefaults();
    const auto value = [&](int id) {
        return inEffect.has[id] ? inEffect.val[id] : defaults.val[id];
    };
    const int32_t exposureUs = value(DelayedControls::EXPOSURE);
    const int32_t gain       = value(DelayedControls::GAIN);
    const int32_t frameLen   = value(DelayedControls::FRAME_LENGTH);
    const int32_t lineLength = value(DelayedControls::LINE_LENGTH);
    const int32_t digital    = value(DelayedControls::DIGITAL_GAIN);
    r.exposureTimeNs  = (int64_t)exposureUs * 1000LL;
    r.sensitivityIso  = cfg.sensitivityIso(gain, digital);
    r.frameDurationNs = cfg.frameDurationNs(frameLen, lineLength);

    if (settings.ae.mode == AeMode::Off)
        r.aeState = AeState::Inactive;
    else if (settings.ae.lock)
        r.aeState = AeState::Locked;
    else if (converged.ae)
        r.aeState = AeState::Converged;
    else
        r.aeState = AeState::Searching;

    if (settings.awb.mode != AwbMode::Auto)
        r.awbState = AwbState::Inactive;
    else if (settings.awb.lock)
        r.awbState = AwbState::Locked;
    else if (converged.awb)
        r.awbState = AwbState::Converged;
    else
        r.awbState = AwbState::Searching;

    r.afState               = af.afState;
    r.afMode                = af.afMode;
    r.focusDistanceDiopters = af.focusDiopter;

    r.cropRegion = settings.crop;
    r.afRegion   = settings.af.region;
    return r;
}

} /* namespace android */
