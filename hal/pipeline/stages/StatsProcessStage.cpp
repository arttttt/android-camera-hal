#include "StatsProcessStage.h"

#include <stdint.h>

#include <system/camera_metadata.h>

#include "PipelineContext.h"
#include "ipa/Ipa.h"
#include "ipa/IpaStats.h"
#include "ipa/StatsWorker.h"
#include "sensor/DelayedControls.h"
#include "sensor/SensorConfig.h"

namespace android {

StatsProcessStage::StatsProcessStage(const Deps &d) : deps(d) {}

void StatsProcessStage::process(PipelineContext &ctx) {
    if (!deps.ipa || !deps.delayedControls || !deps.sensorCfg
        || !deps.statsWorker) return;

    /* Manual AE (AE_MODE_OFF) hands full authority to the framework;
     * ApplySettingsStage pushes the request's exposure / gain directly
     * and the IPA's decision would only clobber it. Skip the push in
     * that mode — the IPA's compute has still run, but its effect
     * lives only in its internal state (useful for a clean switch
     * back to auto). */
    if (ctx.request.settings.exists(ANDROID_CONTROL_AE_MODE)) {
        const uint8_t aeMode =
            *ctx.request.settings.find(ANDROID_CONTROL_AE_MODE).data.u8;
        if (aeMode == ANDROID_CONTROL_AE_MODE_OFF) return;
    }

    IpaStats stats;
    uint32_t statsSeq = 0;
    if (!deps.statsWorker->peek(&stats, &statsSeq)) return;

    /* Pass the frame-of-stats sequence to the IPA so it can correlate
     * with its own history. DelayedControls::push still uses ctx.sequence
     * for effect timing — the control goes into effect relative to
     * today's frame, not to the frame the stats came from. */
    DelayedControls::Batch batch = deps.ipa->processStats(statsSeq, stats);

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

        const uint32_t effectSeq = ctx.sequence
                                   + (uint32_t)deps.sensorCfg->controlDelay[id];
        deps.delayedControls->push(effectSeq, one);
    }
}

} /* namespace android */
