#include "StatsDispatchStage.h"

#include "3a/AutoFocusController.h"
#include "BayerSource.h"
#include "IspPipeline.h"
#include "PipelineContext.h"
#include "Resolution.h"
#include "ipa/IpaStats.h"
#include "ipa/StatsWorker.h"

namespace android {

StatsDispatchStage::StatsDispatchStage(const Deps &d) : deps(d) {}

void StatsDispatchStage::process(PipelineContext &ctx) {
    if (!deps.isp || !deps.statsWorker || !deps.bayerSource) return;
    if (!ctx.bayerFrame) return;
    if (!ctx.status.ok()) return;

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
    /* The worker reads this frame past the pipeline frame's own life
     * (its second phase runs on the next frame's submit): it holds a
     * reference of its own and gives it back when done. */
    deps.bayerSource->retainFrame(ctx.bayerFrame);
    job.hold     = ctx.bayerFrame;
    job.width    = res.width;
    job.height   = res.height;
    job.pixFmt   = ctx.bayerFrame->pixFmt;
    job.sequence = ctx.sequence;
    job.exposureStartNs = ctx.bayerFrame->exposureStartNs();

    /* Pull the latest AF region (set by AutoFocusController::onSettings
     * from ANDROID_CONTROL_AF_REGIONS) into the job. Fallback when AF
     * is null: the compile-time centre rectangle, identical to what
     * the worker used pre-wiring. */
    if (deps.af) {
        const AutoFocusController::FocusRoi roi = deps.af->currentFocusRoi();
        job.focusRoi.pyLo = roi.pyLo;
        job.focusRoi.pyHi = roi.pyHi;
        job.focusRoi.pxLo = roi.pxLo;
        job.focusRoi.pxHi = roi.pxHi;
    } else {
        job.focusRoi.pyLo = IpaStats::FOCUS_ROI_PY_LO;
        job.focusRoi.pyHi = IpaStats::FOCUS_ROI_PY_HI;
        job.focusRoi.pxLo = IpaStats::FOCUS_ROI_PX_LO;
        job.focusRoi.pxHi = IpaStats::FOCUS_ROI_PX_HI;
    }

    deps.statsWorker->submit(job);
}

} /* namespace android */
