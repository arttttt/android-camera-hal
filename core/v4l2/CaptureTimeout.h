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

#ifndef HAL_V4L2_CAPTURE_TIMEOUT_H
#define HAL_V4L2_CAPTURE_TIMEOUT_H

#include <stdint.h>

#include <functional>
#include <vector>

namespace android {

/* How long the VI waits for a frame (V4L2_CID_VI_CAPTURE_TIMEOUT),
 * kept to the frames in flight.
 *
 * The frame period is ours: the HAL writes the frame length, and a
 * write lands `controlDelay` frames later. Until it lands, and until
 * the frames of the old length have been captured, the VI can be
 * waiting on either length; so the timeout covers the longest period
 * written over the last controlDelay + kLatchSlack frames. A longer
 * frame raises it at once -- before its write reaches the sensor --
 * and a shorter one lowers it only once the longer frames have left
 * the window.
 *
 * Not thread-safe: fed by the one thread that writes the sensor. */
class CaptureTimeout {
public:
    /* Writes the timeout, in µs, to the VI. */
    using Writer = std::function<void(int32_t us)>;

    /* `minUs` / `maxUs`: the range the VI takes the control in. */
    CaptureTimeout(Writer writer, int controlDelay,
                   int32_t minUs, int32_t maxUs);

    /* A new stream: every frame in flight is `periodNs` long. Writes
     * the timeout for it. */
    void reset(int64_t periodNs);

    /* A frame of `periodNs` is about to be written to the sensor.
     * Call before the write. */
    void frame(int64_t periodNs);

    /* The timeout last written, µs; 0 before the first. */
    int32_t current() const { return written; }

    /* What the VI waits, for frames up to `periodNs` long: the wait
     * for a frame to start can take a whole period and its readout
     * another, plus slack for scheduling. Never below kFloorUs. */
    static int32_t timeoutUsFor(int64_t periodNs);

    /* Frames a write can land past its control delay. */
    static constexpr int     kLatchSlack = 2;
    static constexpr int32_t kFloorUs    = 200000;
    static constexpr int32_t kSlackUs    = 100000;

private:
    void update();

    Writer               writer;
    const int32_t        minUs;
    const int32_t        maxUs;
    std::vector<int64_t> window;   /* periods, ns, a ring */
    size_t               next;
    int32_t              written;
};

} /* namespace android */

#endif /* HAL_V4L2_CAPTURE_TIMEOUT_H */
