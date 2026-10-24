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

#include "StatsEngine.h"
#include "util/PerfLog.h"

#include <utils/Log.h>
#include <utils/Timers.h>

#undef  LOG_TAG
#define LOG_TAG "Cam-StatsEngine"

namespace mocha::camera {

using android::IpaStats;
using android::NeonStatsEncoder;

StatsEngine::~StatsEngine() {
    stop();
}

bool StatsEngine::start() {
    if (engine)
        return false;
    engine = std::make_unique<Eng>(
            "CamStats",
            [this](Eng::Id, Job &job) {
                runPhase(job);
                return Eng::Step::done();
            });
    return engine->start();
}

void StatsEngine::stop() {
    if (!engine)
        return;
    engine->stop();
    engine.reset();
}

bool StatsEngine::submit(Job job) {
    return engine && engine->submit(std::move(job), DropPolicy::LatestOnly).has_value();
}

bool StatsEngine::peek(Published *out) {
    if (!out) return false;
    std::lock_guard<std::mutex> lock(outLock);
    if (!latestValid) return false;
    *out = latest;
    return true;
}

void StatsEngine::reset() {
    cycle = Job{};
    phase = 0;
    NeonStatsEncoder::resetPartial(&partial);
    std::lock_guard<std::mutex> lock(outLock);
    latestValid = false;
}

void StatsEngine::runPhase(Job &job) {
    /* Idle: a new cycle on this job's frame. Otherwise the job only
     * advances the cycle; its own frame goes with it. */
    if (phase == 0) {
        cycle = std::move(job);
        NeonStatsEncoder::resetPartial(&partial);
    }

    const int rowsPerPhase = IpaStats::PATCH_Y / kPhaseCount;
    const int pyStart      = phase * rowsPerPhase;
    const int pyEnd        = (phase + 1 == kPhaseCount)
                                ? IpaStats::PATCH_Y
                                : (phase + 1) * rowsPerPhase;

    const nsecs_t tNeon0 = systemTime();
    encoder.computeRange(cycle.data, cycle.width, cycle.height,
                         cycle.pixFmt, cycle.focusRoi,
                         &partial, pyStart, pyEnd);
    const nsecs_t tNeon1 = systemTime();

    const int phaseDone = phase;
    ++phase;
    if (phase < kPhaseCount) {
        if (mocha::camera::perfLog())
            ALOGD("PERF: phase=%d/%d neon=%lldus seq=%u",
                  phaseDone + 1, kPhaseCount,
                  (long long)((tNeon1 - tNeon0) / 1000), cycle.sequence);
        return;
    }

    IpaStats result;
    const nsecs_t tFin0 = systemTime();
    NeonStatsEncoder::finalize(partial, cycle.pixFmt, blackLevel, &result);
    const nsecs_t tFin1 = systemTime();
    {
        std::lock_guard<std::mutex> lock(outLock);
        latest.stats           = result;
        latest.sequence        = cycle.sequence;
        latest.exposureStartNs = cycle.exposureStartNs;
        latest.inEffect        = cycle.inEffect;
        latest.meteringRoi     = cycle.meteringRoi;
        latestValid            = true;
    }
    if (mocha::camera::perfLog())
        ALOGD("PERF: phase=%d/%d neon=%lldus finalize=%lldus seq=%u",
              phaseDone + 1, kPhaseCount,
              (long long)((tNeon1 - tNeon0) / 1000),
              (long long)((tFin1  - tFin0)  / 1000), cycle.sequence);
    /* The cycle is done reading its frame. */
    cycle = Job{};
    phase = 0;
}

} /* namespace mocha::camera */
