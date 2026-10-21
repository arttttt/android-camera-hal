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

#ifndef CORE_3A_AWB_CONVERGENCE_H
#define CORE_3A_AWB_CONVERGENCE_H

#include <cmath>

namespace android {

/* Whether AWB has converged: the published (smoothed) gains have
 * reached the tick's target -- what Camera2 reports as CONVERGED, and
 * SEARCHING while they move toward it.
 *
 * With hysteresis: in within kEnter of the target on both R and B, out
 * only past kLeave on either, so the search noise of a settled scene
 * (the Bayes estimate jitters by a few per cent tick to tick, and the
 * damping lags it) does not flicker the state. kEnter matches AE's
 * dead-band. */
class AwbConvergence {
public:
    static constexpr float kEnter = 0.02f;
    static constexpr float kLeave = 0.05f;

    /* One AWB tick: the gains it published and the target it moves
     * them toward. Returns whether AWB is converged now. */
    bool update(float wbR, float wbB, float targetR, float targetB) {
        const float off = std::fmax(offBy(wbR, targetR), offBy(wbB, targetB));
        converged = converged ? off <= kLeave : off <= kEnter;
        return converged;
    }

    bool isConverged() const { return converged; }

    void reset() { converged = false; }

private:
    static float offBy(float value, float target) {
        return target > 0.f ? std::fabs(value / target - 1.0f) : 0.f;
    }

    bool converged = false;
};

} /* namespace android */

#endif /* CORE_3A_AWB_CONVERGENCE_H */
