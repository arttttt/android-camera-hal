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

#ifndef CORE_PIPELINE_FRAME_REGISTRY_H
#define CORE_PIPELINE_FRAME_REGISTRY_H

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <vector>

#include "Frame.h"

namespace mocha::camera {

/* The frames the HAL holds, for flush and close: those it must cancel,
 * wait for, or hand back in error. It knows them, it does not keep them
 * alive on its own -- the jobs and the result engine do; a frame leaves
 * the registry when its result has gone out.
 *
 * The ceiling replaces the blocking queues that used to bound the
 * frames in flight. Camera3 gives process_capture_request no way to
 * refuse a request it was right to send (0, -EINVAL, -ENODEV only), so
 * a frame over the ceiling is still accepted and registered, and the
 * caller fails it at once: it goes out as a failed request, in order. */
class FrameRegistry {
public:
    explicit FrameRegistry(size_t ceiling) : ceiling(ceiling) {}

    FrameRegistry(const FrameRegistry &) = delete;
    FrameRegistry &operator=(const FrameRegistry &) = delete;

    /* False when the frame takes the registry over its ceiling. */
    bool add(const FrameRef &frame);

    void remove(uint32_t frameNumber);

    /* Cancel every frame `which` picks; one already failed or
     * committed keeps its outcome. */
    void cancelIf(const std::function<bool(const Frame &)> &which);

    /* Until no frame is held, up to timeoutMs. True if it emptied. */
    bool waitEmpty(int timeoutMs);

    /* Every frame still held, the registry emptied. */
    std::vector<FrameRef> takeAll();

    size_t count() const;

private:
    const size_t ceiling;
    mutable std::mutex mutex;
    std::condition_variable emptied;
    std::map<uint32_t, FrameRef> frames;
};

} /* namespace mocha::camera */

#endif /* CORE_PIPELINE_FRAME_REGISTRY_H */
