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

#ifndef CORE_SENSOR_SENSOR_SEQUENCE_H
#define CORE_SENSOR_SENSOR_SEQUENCE_H

#include <stdint.h>

#include <functional>

namespace android {

/* Numbers the sensor's frames, from 0 at the first frame of a stream.
 *
 * The capture driver numbers the buffers it completes, damaged ones
 * included, but a sensor frame the VI had no buffer armed for -- every
 * buffer held by the HAL -- passes uncounted, and its number falls
 * behind the sensor's. The buffers' times tell what it missed: from one
 * buffer's end to the next is the frames between them, each as long as
 * the frame and line length in effect on it made it. A frame is as many
 * past the last as the driver counted, and more while the time left
 * holds at least half of the next frame's period.
 *
 * Not thread-safe: fed by the thread that dequeues. */
class SensorSequence {
public:
    /* How long sensor frame `seq` is, ns: the frame and line length in
     * effect on it. */
    using PeriodOf = std::function<int64_t(uint32_t seq)>;

    /* A new stream: the next frame is 0. */
    void reset() { started = false; }

    /* The sensor number of the frame the driver numbered `driverSeq`,
     * its last line in memory at `timestampNs`. */
    uint32_t next(uint32_t driverSeq, int64_t timestampNs,
                  const PeriodOf &periodOf);

    /* Sensor frames the driver did not count since reset(). */
    uint32_t uncounted() const { return missed; }

private:
    bool     started = false;
    uint32_t lastDriver = 0;
    int64_t  lastNs = 0;
    uint32_t lastSeq = 0;
    uint32_t missed = 0;
};

} /* namespace android */

#endif /* CORE_SENSOR_SENSOR_SEQUENCE_H */
