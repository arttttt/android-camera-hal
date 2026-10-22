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

#ifndef CORE_V4L2_SENSOR_WRITER_H
#define CORE_V4L2_SENSOR_WRITER_H

#include "sensor/DelayedControls.h"

namespace android {

struct SensorConfig;
class  V4l2Device;
class  ViController;

/* Puts one frame's controls on the sensor: the VI's frame wait first,
 * then the frame and line length, exposure and gain in one group hold,
 * so the sensor latches them on the same frame.
 *
 * Called on every frame's tick: the VI's wait follows the frames
 * written, changed or not, and the sensor is written only when
 * something changed. The capture engine's thread only. */
class SensorWriter {
public:
    /* `vi` may be null: the VI then waits its default. */
    SensorWriter(V4l2Device *dev, const SensorConfig *cfg, ViController *vi);

    /* `values`: every control set, in effect a control delay from now.
     * Written to the sensor when `changed`. */
    void frame(const DelayedControls::Batch &values, bool changed);

private:
    V4l2Device         *dev;
    const SensorConfig *cfg;
    ViController       *vi;
    /* Whether the driver has V4L2_CID_GROUP_HOLD; asked once. */
    bool                groupHold;
};

} /* namespace android */

#endif /* CORE_V4L2_SENSOR_WRITER_H */
