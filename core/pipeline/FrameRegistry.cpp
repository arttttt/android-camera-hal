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

#include "FrameRegistry.h"

#include <chrono>
#include <utility>

namespace mocha::camera {

bool FrameRegistry::add(const FrameRef &frame) {
    if (!frame)
        return true;
    std::lock_guard<std::mutex> lock(mutex);
    frames[frame->frameNumber] = frame;
    return frames.size() <= ceiling;
}

void FrameRegistry::remove(uint32_t frameNumber) {
    std::lock_guard<std::mutex> lock(mutex);
    frames.erase(frameNumber);
    if (frames.empty())
        emptied.notify_all();
}

void FrameRegistry::cancelIf(const std::function<bool(const Frame &)> &which) {
    std::lock_guard<std::mutex> lock(mutex);
    for (auto &entry : frames)
        if (which(*entry.second))
            entry.second->status.cancel();
}

bool FrameRegistry::waitEmpty(int timeoutMs) {
    std::unique_lock<std::mutex> lock(mutex);
    return emptied.wait_for(lock, std::chrono::milliseconds(timeoutMs),
                            [this] { return frames.empty(); });
}

std::vector<FrameRef> FrameRegistry::takeAll() {
    std::vector<FrameRef> out;
    std::lock_guard<std::mutex> lock(mutex);
    out.reserve(frames.size());
    for (auto &entry : frames)
        out.push_back(std::move(entry.second));
    frames.clear();
    emptied.notify_all();
    return out;
}

size_t FrameRegistry::count() const {
    std::lock_guard<std::mutex> lock(mutex);
    return frames.size();
}

} /* namespace mocha::camera */
