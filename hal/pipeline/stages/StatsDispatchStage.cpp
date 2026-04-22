#include "StatsDispatchStage.h"

#include <stdint.h>

#include "BayerSource.h"
#include "IspPipeline.h"
#include "PipelineContext.h"
#include "Resolution.h"
#include "ipa/StatsWorker.h"

namespace android {

namespace {
/* Submit the Bayer slot to the CPU stats worker every Nth frame.
 * NEON compute is ~7–10 ms on 720p; a 2-frame window gives the
 * worker roughly a full sensor period of slack before the next
 * submit even if the GPU spikes, which prevents backpressure (the
 * latest-wins slot drops old jobs) and frame pacing hiccups on the
 * consumer side. 3A at 30+ Hz is still well above typical 10–15 Hz
 * convergence targets, so this is a pure headroom win. */
constexpr uint32_t STATS_INTERVAL = 2;
} /* namespace */

StatsDispatchStage::StatsDispatchStage(const Deps &d) : deps(d) {}

void StatsDispatchStage::process(PipelineContext &ctx) {
    if (!deps.isp || !deps.statsWorker || !deps.bayerSource) return;
    if (!ctx.bayerFrame) return;
    if (ctx.errorCode)   return;

    if ((ctx.sequence % STATS_INTERVAL) != 0u) return;

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
