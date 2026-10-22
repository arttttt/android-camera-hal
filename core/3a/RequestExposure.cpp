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

#include "RequestExposure.h"

#include <optional>

#include "AutoExposureController.h"
#include "sensor/ExposureTarget.h"
#include "sensor/SensorConfig.h"

namespace android {

DelayedControls::Batch requestExposure(
        const mocha::camera::FrameSettings &settings,
        const SensorConfig                 &cfg,
        const ExposureTarget               *target,
        mocha::camera::Rational             evStep) {
    const std::optional<DelayedControls::Batch> decided =
        (settings.ae.mode != mocha::camera::AeMode::Off && target)
            ? target->latest() : std::nullopt;
    if (decided) {
        DelayedControls::Batch b = *decided;
        const DelayedControls::Batch defaults = cfg.configureDefaults();
        for (int id = 0; id < DelayedControls::COUNT; ++id) {
            if (b.has[id]) continue;
            b.has[id] = true;
            b.val[id] = defaults.val[id];
        }
        return b;
    }

    const ExposureWriteValues v =
        AutoExposureController::parseManualSettings(settings, cfg, evStep);
    DelayedControls::Batch b;
    for (bool &h : b.has) h = true;
    b.val[DelayedControls::EXPOSURE]     = v.exposureUs;
    b.val[DelayedControls::GAIN]         = v.gain;
    b.val[DelayedControls::FRAME_LENGTH] = v.frameLen;
    b.val[DelayedControls::LINE_LENGTH]  = v.lineLength;
    return b;
}

} /* namespace android */
