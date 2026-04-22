#ifndef HAL_PIPELINE_STAGES_STATS_PROCESS_STAGE_H
#define HAL_PIPELINE_STAGES_STATS_PROCESS_STAGE_H

#include "PipelineStage.h"

namespace android {

class  IspPipeline;
class  Ipa;
class  DelayedControls;
struct SensorConfig;

/* Runs on PipelineThread once the frame's submit fence has
 * signalled. Pulls the GPU-written IpaStats off the backend,
 * hands it to the IPA, and publishes the returned control batch
 * into DelayedControls so ApplySettingsStage picks it up on the
 * frame that actually lands the write (seq + controlDelay[id]).
 *
 * Not alwaysRun — on an errored context the stats buffer has no
 * meaningful content and the IPA is skipped.
 *
 * One push per control is issued because each control id carries
 * its own delay and DelayedControls::push tags the whole batch
 * with a single sequence. Today's SensorConfig gives both
 * exposure and gain a delay of 2, so the loop degenerates to a
 * single push, but the shape is future-proof. */
class StatsProcessStage : public PipelineStage {
public:
    struct Deps {
        IspPipeline         *isp;
        Ipa                 *ipa;
        DelayedControls     *delayedControls;
        const SensorConfig  *sensorCfg;
    };

    explicit StatsProcessStage(const Deps &deps);

    const char *name() const override { return "StatsProcess"; }
    void process(PipelineContext &context) override;

private:
    Deps deps;
};

} /* namespace android */

#endif /* HAL_PIPELINE_STAGES_STATS_PROCESS_STAGE_H */
