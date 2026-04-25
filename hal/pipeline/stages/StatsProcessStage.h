#ifndef HAL_PIPELINE_STAGES_STATS_PROCESS_STAGE_H
#define HAL_PIPELINE_STAGES_STATS_PROCESS_STAGE_H

#include "PipelineStage.h"

namespace android {

class  Ipa;
class  DelayedControls;
class  StatsWorker;
class  AutoFocusController;
struct SensorConfig;

/* Runs on PipelineThread once the frame's submit fence has
 * signalled. Peeks the latest IpaStats the StatsWorker has
 * published (computed off-thread in parallel with the GPU submit),
 * hands it to the IPA, and publishes the returned control batch
 * into DelayedControls so ApplySettingsStage picks it up on the
 * frame that actually lands the write (seq + controlDelay[id]).
 *
 * The same stats buffer also feeds AutoFocusController's sweep
 * scoring — the per-patch sharpness grid is already computed and
 * AF only needs a small reduction over the centre, so doing it
 * here keeps everything that consumes IPA stats on one thread
 * while the buffer is hot.
 *
 * Not alwaysRun — errored contexts skip the IPA since a control
 * update derived from stats we did not produce for this frame
 * adds no value.
 *
 * One push per control is issued because each control id carries
 * its own delay and DelayedControls::push tags the whole batch
 * with a single sequence. Today's SensorConfig gives both exposure
 * and gain a delay of 2, so the loop degenerates to a single
 * push, but the shape is future-proof. */
class StatsProcessStage : public PipelineStage {
public:
    struct Deps {
        Ipa                 *ipa;
        DelayedControls     *delayedControls;
        const SensorConfig  *sensorCfg;
        StatsWorker         *statsWorker;
        AutoFocusController *af;          /* may be null */
    };

    explicit StatsProcessStage(const Deps &deps);

    const char *name() const override { return "StatsProcess"; }
    void process(PipelineContext &context) override;

private:
    Deps deps;
};

} /* namespace android */

#endif /* HAL_PIPELINE_STAGES_STATS_PROCESS_STAGE_H */
