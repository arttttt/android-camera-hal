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
        /* AE decided for the range of the frame it last saw; this
         * request's own range bounds its frame. Too long a frame:
         * the same exposure divided again within it. Too short: the
         * frame lengthened, the exposure as it was. */
        const SensorConfig::FrameLimits limits = cfg.frameLimits(settings.ae.fpsRange);
        int32_t &frameLen = b.val[DelayedControls::FRAME_LENGTH];
        if (frameLen > limits.maxLen) {
            const float totalUs = (float)b.val[DelayedControls::EXPOSURE]
                * (float)b.val[DelayedControls::GAIN] / (float)cfg.gainUnit
                * (float)b.val[DelayedControls::DIGITAL_GAIN]
                / (float)SensorConfig::kDigitalGainUnit;
            const SensorConfig::ExposureSplit split = cfg.splitExposure(
                totalUs, limits, cfg.flickerLines(settings.ae.antibanding));
            b.val[DelayedControls::EXPOSURE]     = split.exposureUs;
            b.val[DelayedControls::GAIN]         = split.gain;
            b.val[DelayedControls::DIGITAL_GAIN] = split.digitalGain;
            frameLen                             = split.frameLen;
        } else if (frameLen < limits.minLen) {
            frameLen = limits.minLen;
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
    b.val[DelayedControls::DIGITAL_GAIN] = v.digitalGain;
    return b;
}

} /* namespace android */
