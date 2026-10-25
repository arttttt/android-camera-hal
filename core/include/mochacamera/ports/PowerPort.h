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

#ifndef MOCHACAMERA_PORTS_POWER_PORT_H
#define MOCHACAMERA_PORTS_POWER_PORT_H

#include <cstdint>

namespace mocha::camera {

/* Where the core says how much work it is about to do, so the system
 * can hold its clocks up for it. The core says when and how much, in
 * its own terms; what that becomes -- power hints, nothing at all --
 * is the frontend's. With no port the core says nothing.
 *
 * Calls come from the session's callers (open, close, configure,
 * submit) and must not block. */
class PowerPort {
public:
    virtual ~PowerPort() = default;

    /* The session is opening: a short burst until the stream runs. */
    virtual void opening() = 0;

    /* Frames are coming at this rate of sensor pixels, until stopped();
     * a new rate replaces the last. */
    virtual void streaming(uint64_t pixelsPerSecond) = 0;

    /* A still of this many pixels is on its way to its JPEG. */
    virtual void stillCapture(uint64_t pixels) = 0;

    /* No frames are coming: the stream stopped or the session closed. */
    virtual void stopped() = 0;
};

} /* namespace mocha::camera */

#endif /* MOCHACAMERA_PORTS_POWER_PORT_H */
