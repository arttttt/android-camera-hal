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

#ifndef V4L2_SENSOR_SUBDEV_H
#define V4L2_SENSOR_SUBDEV_H

#include <optional>
#include <string>
#include <vector>

#include "mochacamera/Controls.h"
#include "sensor/SensorGeometry.h"

namespace android {

/* The sensor's geometry as its driver reports it on its subdev node:
 * the pixel array (NATIVE_SIZE), and for each of `outputs` the window
 * of the array its mode shows (the CROP of a TRY format, which leaves
 * the sensor's mode as it is). None when the driver reports no pixel
 * array; an output whose window it does not report is left out. */
std::optional<SensorGeometry> readSensorGeometry(
        const std::string &subdevNode,
        const std::vector<mocha::camera::Size> &outputs);

} /* namespace android */

#endif /* V4L2_SENSOR_SUBDEV_H */
