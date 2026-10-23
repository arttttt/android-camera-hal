#include "StatsProcessStage.h"

#include <stdint.h>

#include <utils/Log.h>
#include <utils/Timers.h>

#include "Frame.h"
#include "ipa/Ipa.h"
#include "ipa/IpaFrameMeta.h"
#include "ipa/IpaStats.h"
#include "StatsEngine.h"
#include "sensor/DelayedControls.h"
#include "sensor/ExposureTarget.h"
#include "sensor/SensorConfig.h"

#undef  LOG_TAG
#define LOG_TAG "Cam-StatsProcessStage"

namespace android {

StatsProcessStage::StatsProcessStage(const Deps &d) : deps(d) {}

void StatsProcessStage::process(mocha::camera::Frame &ctx) {
    if (!deps.ipa || !deps.exposureTarget || !deps.sensorCfg
        || !deps.stats) return;

    const nsecs_t t0 = systemTime();

    mocha::camera::StatsEngine::Published published;
    if (!deps.stats->peek(&published)) return;
    const IpaStats &stats = published.stats;

    const nsecs_t tPeek = systemTime();

    /* The frame's 3A modes and locks. Gating per control (skip
     * exposure / gain push when AE is Off, skip WB push when AWB is
     * Off or locked) lives inside the IPA, so AE and AWB can be driven
     * independently — AE Off with AWB Auto still lets WB update, and
     * vice versa. */
    const mocha::camera::FrameSettings &s = ctx.settings;
    IpaFrameMeta meta;
    meta.aeMode                 = s.ae.mode;
    meta.aeLock                 = s.ae.lock;
    meta.awbMode                = s.awb.mode;
    meta.awbLock                = s.awb.lock;
    meta.aeExposureCompensation = s.ae.evCompensationSteps;

    /* Where AE / AWB meter: what the stats' own frame asked -- its AE
     * region, or where its AF looked (CaptureEngine::dispatchStats). */
    meta.meteringRoi = published.meteringRoi;

    /* Manual WB: the request's gains, which the frontend passes on for
     * AWB Off only (in Auto the framework round-trips the published
     * gains, and reading them would freeze AWB). */
    if (s.awb.manualGains) {
        meta.manualWbValid = true;
        meta.manualWbR = s.awb.manualGains->r;
        meta.manualWbG = s.awb.manualGains->g;
        meta.manualWbB = s.awb.manualGains->b;
    }

    /* Pass the frame-of-stats sequence to the IPA so it can correlate
     * with its own history.
     *
     * IPA also drives AF internally now (post-step-5 of the 3A
     * refactor) — the coordinator calls af->process(stats, ...) and
     * routes its result, so this stage no longer touches AF.
     *
     * IpaProcessParams carries everything the AWB / AE partial results
     * inside the tick need (frameNumber, timestampNs, what was in
     * effect, the frame's settings). */
    IpaProcessParams params = {
        published.sequence,
        published.exposureStartNs,
        published.inEffect,
        ctx.frameNumber,
        ctx.exposureStartNs ? ctx.exposureStartNs : ctx.tAccepted,
        stats,
        meta,
        ctx.settings,
        *deps.sensorCfg,
        ctx.inEffect,
        deps.sink,
    };
    DelayedControls::Batch batch = deps.ipa->processStats(params);
    ctx.aeConverged  = deps.ipa->isAeConverged();
    ctx.awbConverged = deps.ipa->isAwbConverged();

    const nsecs_t tIpa = systemTime();

    /* The decision is what auto exposure wants now; the next request
     * in auto AE takes it. */
    deps.exposureTarget->publish(batch);

    const nsecs_t tEnd = systemTime();
    ALOGD("PERF: peek=%lldus ipa=%lldus publish=%lldus total=%lldus f=%u",
          (long long)((tPeek - t0)    / 1000),
          (long long)((tIpa  - tPeek) / 1000),
          (long long)((tEnd  - tIpa)  / 1000),
          (long long)((tEnd  - t0)    / 1000),
          ctx.frameNumber);
}

} /* namespace android */
