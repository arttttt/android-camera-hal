#include "StatsDispatchStage.h"

#include <stdint.h>

#include "BayerSource.h"
#include "IspPipeline.h"
#include "PipelineContext.h"
#include "Resolution.h"
#include "ipa/StatsWorker.h"

namespace android {

namespace {
/* Gate Bayer submits to give the CPU stats worker a multi-frame
 * deadline. The effective stats compute budget is
 *
 *     t_stats = frame_time × STATS_INTERVAL
 *
 * At INTERVAL=2 and 60–90 fps sensor cadence NEON has roughly
 * 22–33 ms to finish one pass before the next submit arrives.
 * Today's compute (128-bin luma histogram + 16×16 patch RGB mean
 * + per-patch Tenengrad on green) runs in ~8 ms on 720p, so the
 * budget is mostly headroom; heavier stats shapes (per-channel
 * histograms, finer patch grid, per-patch variance) in the future
 * can grow inside the same deadline without restructuring the
 * worker.
 *
 * Setting INTERVAL=1 would double the stats rate at the cost of
 * halving the deadline and raising peak CPU. 3A convergence targets
 * 10–15 Hz updates; sensor/INTERVAL at any reasonable INTERVAL is
 * already well above that. */
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
