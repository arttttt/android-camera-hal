#ifndef HAL_PIPELINE_STAGES_STATS_DISPATCH_STAGE_H
#define HAL_PIPELINE_STAGES_STATS_DISPATCH_STAGE_H

#include "PipelineStage.h"

namespace android {

class BayerSource;
class IspPipeline;
class StatsWorker;

/* Runs on RequestThread immediately after CaptureStage. Pulls the raw
 * Bayer pointer for the ctx's V4L2 slot out of the ISP backend and
 * hands it to the StatsWorker. The worker computes IpaStats in
 * parallel with PipelineThread's Vulkan demosaic + blit submit, so
 * the CPU stats cost no longer serialises with the GPU frame time.
 *
 * Not alwaysRun — on an errored ctx we skip the submit, matching the
 * downstream StatsProcessStage guard that only consumes fresh stats
 * from non-errored frames. */
class StatsDispatchStage : public PipelineStage {
public:
    struct Deps {
        IspPipeline *isp;
        StatsWorker *statsWorker;
        BayerSource *bayerSource;
    };

    explicit StatsDispatchStage(const Deps &deps);

    const char *name() const override { return "StatsDispatch"; }
    void process(PipelineContext &context) override;

private:
    Deps deps;
};

} /* namespace android */

#endif /* HAL_PIPELINE_STAGES_STATS_DISPATCH_STAGE_H */
