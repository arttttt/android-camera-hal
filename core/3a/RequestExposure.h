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

#ifndef CORE_3A_REQUEST_EXPOSURE_H
#define CORE_3A_REQUEST_EXPOSURE_H

#include "mochacamera/Controls.h"
#include "mochacamera/FrameSettings.h"
#include "sensor/DelayedControls.h"

namespace android {

class  ExposureTarget;
struct SensorConfig;

/* What a request asks of the sensor, every control set.
 *
 * Auto AE: the IPA's latest decision (ExposureTarget); a control it has
 * never decided at what configure wrote.
 * Manual AE, or auto before the IPA's first decision (cold start, or
 * StubIpa): the request's own settings through
 * AutoExposureController::parseManualSettings, so the sensor is never
 * left frozen. `target` may be null: manual settings only. */
DelayedControls::Batch requestExposure(
        const mocha::camera::FrameSettings &settings,
        const SensorConfig                 &cfg,
        const ExposureTarget               *target,
        mocha::camera::Rational             evStep);

} /* namespace android */

#endif /* CORE_3A_REQUEST_EXPOSURE_H */
