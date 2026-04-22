#include "StatsProcessStage.h"

#include <stdint.h>

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
