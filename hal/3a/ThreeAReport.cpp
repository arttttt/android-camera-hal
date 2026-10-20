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
using mocha::camera::AfState;
using mocha::camera::AwbMode;
using mocha::camera::AwbState;
using mocha::camera::Core3AResult;
using mocha::camera::FrameSettings;

Core3AResult describe3A(const FrameSettings &settings,
                        const AutoFocusController::Report &af,
                        bool aeConverged,
                        uint32_t frameNumber,
                        const DelayedControls *delayedControls,
                        const SensorConfig &cfg) {
    Core3AResult r;

    int32_t exposureUs = cfg.exposureDefault;
    int32_t gain       = cfg.gainDefault;
    if (delayedControls) {
        int32_t fallback[DelayedControls::COUNT];
        fallback[DelayedControls::EXPOSURE] = cfg.exposureDefault;
        fallback[DelayedControls::GAIN]     = cfg.gainDefault;
        const DelayedControls::Batch live =
            delayedControls->applyControls(frameNumber, fallback);
        exposureUs = live.val[DelayedControls::EXPOSURE];
        gain       = live.val[DelayedControls::GAIN];
    }
    r.exposureTimeNs  = (int64_t)exposureUs * 1000LL;
    r.sensitivityIso  = cfg.gainToIso(gain);
    r.frameDurationNs = (int64_t)cfg.frameLenDefault * cfg.lineTimeUs * 1000LL;

    if (settings.ae.mode == AeMode::Off)
        r.aeState = AeState::Inactive;
    else if (settings.ae.lock)
        r.aeState = AeState::Locked;
    else if (aeConverged)
        r.aeState = AeState::Converged;
    else
        r.aeState = AeState::Searching;

    if (settings.awb.mode == AwbMode::Auto) {
        const bool afSweeps = af.afState == AfState::ActiveScan ||
                              af.afState == AfState::PassiveScan;
        r.awbState = (afSweeps || settings.awb.lock) ? AwbState::Locked
                                                     : AwbState::Converged;
    } else {
        r.awbState = AwbState::Inactive;
    }

    r.afState               = af.afState;
    r.afMode                = af.afMode;
    r.focusDistanceDiopters = af.focusDiopter;
    return r;
}

} /* namespace android */
