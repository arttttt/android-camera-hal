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

#include "CaptureTimeout.h"

#include <algorithm>

namespace android {

CaptureTimeout::CaptureTimeout(Writer w, int controlDelay,
                               int32_t mn, int32_t mx)
    : writer(std::move(w)),
      minUs(mn),
      maxUs(mx),
      window((size_t)std::max(controlDelay, 0) + 1 + kLatchSlack, 0),
      next(0),
      written(0) {
}

int32_t CaptureTimeout::timeoutUsFor(int64_t periodNs) {
    const int64_t us = 2 * (periodNs / 1000) + kSlackUs;
    return (int32_t)std::min<int64_t>(std::max<int64_t>(us, kFloorUs), INT32_MAX);
}

void CaptureTimeout::reset(int64_t periodNs) {
    std::fill(window.begin(), window.end(), periodNs);
    next    = 0;
    written = 0;
    update();
}

void CaptureTimeout::frame(int64_t periodNs) {
    window[next] = periodNs;
    next = (next + 1) % window.size();
    update();
}

void CaptureTimeout::update() {
    const int64_t longest = *std::max_element(window.begin(), window.end());
    const int32_t us = std::clamp(timeoutUsFor(longest), minUs, maxUs);
    if (us == written) return;
    writer(us);
    written = us;
}

} /* namespace android */
