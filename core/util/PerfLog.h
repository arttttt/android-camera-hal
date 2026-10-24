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

#ifndef CORE_UTIL_PERF_LOG_H
#define CORE_UTIL_PERF_LOG_H

#include <atomic>

namespace mocha::camera {

/* Whether the per-frame PERF lines and the GPU timing are on. Off by
 * default: performance comes first, and what is off costs one relaxed
 * load where the line would have been -- nothing is formatted, nothing
 * goes to the log, nothing is recorded on the GPU. Set from
 * Session::Config before the engines start. */
inline std::atomic<bool> &perfLogFlag() {
    static std::atomic<bool> on{false};
    return on;
}

inline bool perfLog() { return perfLogFlag().load(std::memory_order_relaxed); }

inline void setPerfLog(bool on) { perfLogFlag().store(on, std::memory_order_relaxed); }

} /* namespace mocha::camera */

#endif /* CORE_UTIL_PERF_LOG_H */
