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

#ifndef CORE_ENGINES_STATS_ENGINE_H
#define CORE_ENGINES_STATS_ENGINE_H

#include <cstdint>
#include <memory>
#include <mutex>
#include <variant>

#include "mochacamera/ports/FrameSource.h"
#include "Engine.h"
#include "IpaStats.h"
#include "NeonStatsEncoder.h"
#include "sensor/DelayedControls.h"

namespace mocha::camera {

/* Reduces raw Bayer into IpaStats with a progressive NEON kernel, off
 * the GPU's path, for 3A.
 *
 * Progressive cadence: one full IpaStats spans kPhaseCount jobs. The
 * job that finds the engine idle starts a cycle on its own frame; each
 * job after it advances the cycle by one phase -- 1/kPhaseCount of the
 * patch rows -- on that same frame, and the last phase publishes. Peak
 * CPU per frame stays at about a phase's share while one result costs
 * kPhaseCount frame periods.
 *
 * Always the freshest frame: jobs are queued LatestOnly, so a newer one
 * supersedes any still waiting, and the waiting one's lease goes with
 * it. The cycle holds a lease on its frame for its whole span, so the
 * slot it reads cannot go back to the sensor under it.
 *
 * The cycle and the published stats live in this component, across
 * restarts of the engine; reset() clears them between sessions. */
class StatsEngine {
public:
    struct Job {
        BayerLease  bayer;
        const void *data;      /* the frame's CPU view, for the kernel */
        unsigned    width;
        unsigned    height;
        uint32_t    pixFmt;
        uint32_t    sequence;
        /* When the frame's exposure began (systemTime; 0 = unknown). */
        int64_t     exposureStartNs;
        /* The sensor controls in effect on the frame. */
        android::DelayedControls::Batch inEffect;
        /* Patch-grid window the kernel restricts Sobel / greenSq to;
         * taken from AF at submit time. */
        android::NeonStatsEncoder::FocusRoi focusRoi;
    };

    static constexpr int kPhaseCount = 2;

    StatsEngine() = default;
    ~StatsEngine();

    StatsEngine(const StatsEngine &) = delete;
    StatsEngine &operator=(const StatsEngine &) = delete;

    /* Sensor optical-black bias, subtracted as the demosaic shader does,
     * so AE and AWB see the signal the preview shows. 0 disables it.
     * Set while the engine is not running. */
    void setBlackLevel(uint32_t level) { blackLevel = level; }

    bool start();
    void stop();

    /* Never waits. Supersedes a job still waiting. False once stopped. */
    bool submit(Job job);

    /* Stats as published, with what the IPA needs of their frame. */
    struct Published {
        android::IpaStats stats{};
        uint32_t          sequence = 0;
        int64_t           exposureStartNs = 0;   /* 0 = unknown */
        android::DelayedControls::Batch inEffect;
    };

    /* The latest published stats. False before the first publication.
     * Any thread. */
    bool peek(Published *out);

    /* Drop the cycle and the publication, between sessions. The engine
     * must not be running. */
    void reset();

private:
    using Eng = Engine<Job, std::monostate>;

    void runPhase(Job &job);

    android::NeonStatsEncoder encoder;
    uint32_t blackLevel = 0;

    /* The engine thread's own while it runs. */
    Job  cycle{};
    int  phase = 0;
    android::NeonStatsEncoder::Partial partial{};

    std::mutex          outLock;
    Published           latest;
    bool                latestValid = false;

    std::unique_ptr<Eng> engine;
};

} /* namespace mocha::camera */

#endif /* CORE_ENGINES_STATS_ENGINE_H */
