#include "StatsDispatchStage.h"

#include "BayerSource.h"
#include "IspPipeline.h"
#include "PipelineContext.h"
#include "Resolution.h"
#include "ipa/StatsWorker.h"

namespace android {

StatsDispatchStage::StatsDispatchStage(const Deps &d) : deps(d) {}

void StatsDispatchStage::process(PipelineContext &ctx) {
    if (!deps.isp || !deps.statsWorker || !deps.bayerSource) return;
    if (!ctx.bayerFrame) return;
    if (ctx.errorCode)   return;

    /* Submit every frame. Temporal pacing lives inside StatsWorker: one
     * full IpaStats computation spans phaseCount submits, and each
     * submit advances one phase's worth of patch-row work. Throttling
     * here would only duplicate that logic and idle the worker. */
    const int slot = ctx.bayerFrame->index;
    deps.isp->invalidateBayer(slot);
    const void *bayer = deps.isp->bayerHost(slot);
    if (!bayer) return;

    const Resolution res = deps.bayerSource->resolution();

    StatsWorker::Job job;
    job.bayer    = bayer;
    job.width    = res.width;
    job.height   = res.height;
    job.pixFmt   = ctx.bayerFrame->pixFmt;
    job.sequence = ctx.sequence;
    deps.statsWorker->submit(job);
}

} /* namespace android */
