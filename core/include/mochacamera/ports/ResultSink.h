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

#ifndef MOCHACAMERA_PORTS_RESULT_SINK_H
#define MOCHACAMERA_PORTS_RESULT_SINK_H

#include <cstdint>
#include <span>

#include "mochacamera/Buffers.h"
#include "mochacamera/Results.h"

namespace mocha::camera {

/* Where the core sends what it has to say about frames. The frontend
 * implements it -- with Camera3, notify() and process_capture_result()
 * -- and the core never calls the framework any other way.
 *
 * Per frame, in this order: shutter() once; partial() for each of
 * PartialResult unless the frame failed before 3A ran; frame() once,
 * last. Frames reach frame() in the order they were submitted.
 * deviceError() at most once per session, from any thread. */
class ResultSink {
public:
    virtual ~ResultSink() = default;

    virtual void shutter(uint32_t frameNumber, int64_t timestampNs) = 0;

    virtual void partial(uint32_t frameNumber, PartialResult which,
                         const Core3AResult &result) = 0;

    /* The frame's final result and its buffers. Ownership of every
     * release fence passes to the sink. */
    virtual void frame(uint32_t frameNumber, const CoreFrameResult &result,
                       std::span<const BufferReturn> buffers) = 0;

    virtual void deviceError(const char *why) = 0;
};

} /* namespace mocha::camera */

#endif /* MOCHACAMERA_PORTS_RESULT_SINK_H */
