#include "StatsProcessStage.h"

#include <stdint.h>

#include <utils/Log.h>
#include <utils/Timers.h>

#include "Frame.h"
#include "3a/AutoFocusController.h"
#include "ipa/Ipa.h"
#include "ipa/IpaFrameMeta.h"
#include "ipa/IpaStats.h"
#include "StatsEngine.h"
#include "sensor/DelayedControls.h"
#include "sensor/SensorConfig.h"

#undef  LOG_TAG
#define LOG_TAG "Cam-StatsProcessStage"

namespace android {

StatsProcessStage::StatsProcessStage(const Deps &d) : deps(d) {}

void StatsProcessStage::process(mocha::camera::Frame &ctx) {
    if (!deps.ipa || !deps.delayedControls || !deps.sensorCfg
        || !deps.stats) return;

    const nsecs_t t0 = systemTime();

    IpaStats stats;
    uint32_t statsSeq = 0;
    int64_t  statsStartNs = 0;
    if (!deps.stats->peek(&stats, &statsSeq, &statsStartNs)) return;

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

    /* Pull the live AF region into meta so AE / AWB metering follow
     * the user's tap instead of always centring on the frame middle.
     * No tap → AutoFocusController's default rectangle is the same
     * centre 8×8 IpaFrameMeta initialises with, so the metric is
     * unchanged from the centre-weighted default. */
    if (deps.af) {
        const AutoFocusController::FocusRoi roi = deps.af->currentFocusRoi();
        meta.focusRoiPyLo = roi.pyLo;
        meta.focusRoiPyHi = roi.pyHi;
        meta.focusRoiPxLo = roi.pxLo;
        meta.focusRoiPxHi = roi.pxHi;
    }

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
     * with its own history. DelayedControls::push still uses ctx.frameNumber
     * for effect timing — the control goes into effect relative to
     * today's frame, not to the frame the stats came from.
     *
     * IPA also drives AF internally now (post-step-5 of the 3A
     * refactor) — the coordinator calls af->process(stats, ...) and
     * routes its result, so this stage no longer touches AF.
     *
     * IpaProcessParams carries everything the AWB / AE partial results
     * inside the tick need (frameNumber, timestampNs, applied-state
     * lookup via DelayedControls, the frame's settings). */
    IpaProcessParams params = {
        statsSeq,
        statsStartNs,
        ctx.frameNumber,
        ctx.tShutter ? ctx.tShutter : ctx.tAccepted,
        stats,
        meta,
        ctx.settings,
        *deps.sensorCfg,
        deps.delayedControls,
        deps.sink,
    };
    DelayedControls::Batch batch = deps.ipa->processStats(params);
    ctx.aeConverged  = deps.ipa->isAeConverged();
    ctx.awbConverged = deps.ipa->isAwbConverged();

    const nsecs_t tIpa = systemTime();

    /* Publish each set control at seq + its own silicon delay.
     * DelayedControls::push tags the whole batch with one sequence,
     * so one push per control handles the general case where the
     * per-control delays differ. */
    for (int id = 0; id < DelayedControls::COUNT; ++id) {
        if (!batch.has[id]) continue;
        DelayedControls::Batch one;
        for (int i = 0; i < DelayedControls::COUNT; ++i) {
            one.has[i] = false;
            one.val[i] = 0;
        }
        one.has[id] = true;
        one.val[id] = batch.val[id];

        const uint32_t effectSeq = ctx.frameNumber
                                   + (uint32_t)deps.sensorCfg->controlDelay[id];
        deps.delayedControls->push(effectSeq, one);
    }

    const nsecs_t tEnd = systemTime();
    ALOGD("PERF: peek=%lldus ipa=%lldus push=%lldus total=%lldus f=%u",
          (long long)((tPeek - t0)    / 1000),
          (long long)((tIpa  - tPeek) / 1000),
          (long long)((tEnd  - tIpa)  / 1000),
          (long long)((tEnd  - t0)    / 1000),
          ctx.frameNumber);
}

} /* namespace android */
