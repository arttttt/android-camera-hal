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

#ifndef CORE_SESSION_CAPABILITY_PROBE_H
#define CORE_SESSION_CAPABILITY_PROBE_H

#include "mochacamera/Capability.h"

namespace android {
class SensorTuning;
class V4l2Device;
struct SensorConfig;
}

namespace mocha::camera {

/* What the camera can do, from the driver (sizes, frame intervals,
 * exposure and gain ranges), the sensor's tuning (optics, colour
 * filter, AWB presets) and the pipeline's own limits (the output
 * sizes the SW ISP sustains, one YUV size per capture). */
Capability probeCapability(android::V4l2Device &dev, bool facingFront,
                           const android::SensorTuning *tuning,
                           const android::SensorConfig &sensorCfg);

} /* namespace mocha::camera */

#endif /* CORE_SESSION_CAPABILITY_PROBE_H */
