#include "StatsProcessStage.h"

#include <stdint.h>

#include "BayerSource.h"
#include "IspPipeline.h"
#include "PipelineContext.h"
#include "Resolution.h"
#include "ipa/Ipa.h"
#include "ipa/IpaStats.h"
#include "ipa/NeonStatsEncoder.h"
#include "sensor/DelayedControls.h"
#include "sensor/SensorConfig.h"

namespace android {

StatsProcessStage::StatsProcessStage(const Deps &d) : deps(d) {}

void StatsProcessStage::process(PipelineContext &ctx) {
    if (!deps.isp || !deps.ipa || !deps.delayedControls || !deps.sensorCfg
        || !deps.bayerSource || !deps.neonStats) return;
    if (!ctx.bayerFrame) return;

    const int slot = ctx.bayerFrame->index;
    deps.isp->invalidateBayer(slot);
    const void *bayer = deps.isp->bayerHost(slot);
    if (!bayer) return;

    const Resolution res = deps.bayerSource->resolution();

    IpaStats stats;
    deps.neonStats->compute(bayer, res.width, res.height,
                            ctx.bayerFrame->pixFmt, &stats);

    DelayedControls::Batch batch = deps.ipa->processStats(ctx.sequence, stats);

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
