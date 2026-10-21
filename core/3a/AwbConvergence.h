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
 * reached the target the controller moves them toward -- what Camera2
 * reports as CONVERGED, and SEARCHING while they move.
 *
 * Judged on the trend, not the tick: the target is a noisy estimate
 * (the Bayes search steps between neighbouring CT points and its fine
 * search adds more, several per cent tick to tick), and the smoothed
 * gains are its moving average, so on a settled scene the gap between
 * them changes sign and averages out, while a new light keeps it one
 * way. The gap is averaged signed (kTrend a tick, ~5 ticks), the first
 * tick after a reset taking it whole; converged once the average is
 * within kEnter on both R and B, searching again only past kLeave.
 * kEnter matches AE's dead-band. */
class AwbConvergence {
public:
    static constexpr float kEnter = 0.02f;
    static constexpr float kLeave = 0.05f;
    static constexpr float kTrend = 0.2f;

    /* One AWB tick: the gains it published and the target it moves
     * them toward. Returns whether AWB is converged now. */
    bool update(float wbR, float wbB, float targetR, float targetB) {
        const float gapR = gap(wbR, targetR);
        const float gapB = gap(wbB, targetB);
        if (primed) {
            trendR = kTrend * gapR + (1.0f - kTrend) * trendR;
            trendB = kTrend * gapB + (1.0f - kTrend) * trendB;
        } else {
            trendR = gapR;
            trendB = gapB;
            primed = true;
        }
        const float off = std::fmax(std::fabs(trendR), std::fabs(trendB));
        converged = converged ? off <= kLeave : off <= kEnter;
        return converged;
    }

    bool isConverged() const { return converged; }

    void reset() {
        converged = false;
        primed    = false;
        trendR    = 0.f;
        trendB    = 0.f;
    }

private:
    static float gap(float value, float target) {
        return value > 0.f ? target / value - 1.0f : 0.f;
    }

    bool  converged = false;
    bool  primed    = false;
    float trendR    = 0.f;
    float trendB    = 0.f;
};

} /* namespace android */

#endif /* CORE_3A_AWB_CONVERGENCE_H */
