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

#ifndef HAL_3A_THREE_A_REPORT_H
#define HAL_3A_THREE_A_REPORT_H

#include <stdint.h>

#include "AutoFocusController.h"
#include "mochacamera/FrameSettings.h"
#include "mochacamera/Results.h"

namespace android {

class DelayedControls;
struct SensorConfig;

/* What 3A shows the framework for one frame: the states derived from
 * the frame's own modes and locks, AF's report, and the sensor values
 * in effect on it -- what was written a control delay before, or the
 * defaults configureStreams wrote before the first write landed.
 *
 * The one derivation of the states, for the partial results of the 3A
 * tick and for the final result alike:
 *   AE:  Off -> Inactive; locked -> Locked; converged -> Converged;
 *        otherwise Searching.
 *   AWB: Auto -> Locked while locked or while AF sweeps (AF holds it),
 *        otherwise Converged; any other mode -> Inactive. */
mocha::camera::Core3AResult describe3A(
        const mocha::camera::FrameSettings &settings,
        const AutoFocusController::Report  &af,
        bool                                aeConverged,
        uint32_t                            frameNumber,
        const DelayedControls              *delayedControls,
        const SensorConfig                 &sensorCfg);

} /* namespace android */

#endif /* HAL_3A_THREE_A_REPORT_H */
