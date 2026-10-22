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

#ifndef CORE_SENSOR_EXPOSURE_TARGET_H
#define CORE_SENSOR_EXPOSURE_TARGET_H

#include <mutex>
#include <optional>

#include "sensor/DelayedControls.h"

namespace android {

/* What auto exposure wants on the sensor now: the IPA's latest
 * decision, control by control. A request in auto AE takes it as it is
 * when its settings are made; the decision is the IPA's view of the
 * scene, not of any one request, so it is not filed under a request's
 * number.
 *
 * A control the IPA has not decided keeps its last decision; nothing
 * before the first. Published on the finish engine, read on the capture
 * engine. */
class ExposureTarget {
public:
    /* Merges the controls set in `batch` over the target. */
    void publish(const DelayedControls::Batch &batch) {
        std::lock_guard<std::mutex> lock(mutex);
        for (int id = 0; id < DelayedControls::COUNT; ++id) {
            if (!batch.has[id]) continue;
            target.has[id] = true;
            target.val[id] = batch.val[id];
        }
    }

    /* The target; nothing until the IPA has decided a control. */
    std::optional<DelayedControls::Batch> latest() const {
        std::lock_guard<std::mutex> lock(mutex);
        for (int id = 0; id < DelayedControls::COUNT; ++id)
            if (target.has[id]) return target;
        return std::nullopt;
    }

    /* A new session starts from the defaults configure writes, not
     * from where the last one ended. */
    void reset() {
        std::lock_guard<std::mutex> lock(mutex);
        target = DelayedControls::Batch{};
    }

private:
    mutable std::mutex     mutex;
    DelayedControls::Batch target;
};

} /* namespace android */

#endif /* CORE_SENSOR_EXPOSURE_TARGET_H */
