#ifndef HAL_PIPELINE_STAGES_STATS_PROCESS_STAGE_H
#define HAL_PIPELINE_STAGES_STATS_PROCESS_STAGE_H

#include "PipelineStage.h"

namespace mocha::camera {
class ResultSink;
class StatsEngine;
}

namespace android {

class  AutoFocusController;
class  ExposureTarget;
class  Ipa;
struct SensorConfig;

/* Runs on the finish engine once the frame's submit fence has
 * signalled. Peeks the latest IpaStats the stats engine has
 * published (computed off-thread in parallel with the GPU submit),
 * hands it to the IPA, and publishes the returned control batch as
 * the exposure target, which the next request in auto AE takes.
 *
 * The same stats buffer also feeds AutoFocusController's sweep
 * scoring — the per-patch sharpness grid is already computed and
 * AF only needs a small reduction over the centre, so doing it
 * here keeps everything that consumes IPA stats on one thread
 * while the buffer is hot.
 *
 * Not alwaysRun — errored contexts skip the IPA since a control
 * update derived from stats we did not produce for this frame
 * adds no value. */
class StatsProcessStage : public PipelineStage {
public:
    struct Deps {
        Ipa                 *ipa;
        ExposureTarget      *exposureTarget;
        const SensorConfig  *sensorCfg;
        mocha::camera::StatsEngine *stats;
        AutoFocusController *af;          /* may be null — used for
                                           * currentFocusRoi() only       */
        mocha::camera::ResultSink *sink;  /* threaded into the IPA tick so
                                           * the AWB / AE partial results
                                           * go out on this stage         */
    };

    explicit StatsProcessStage(const Deps &deps);

    const char *name() const override { return "StatsProcess"; }
    void process(mocha::camera::Frame &context) override;

private:
    Deps deps;
};

} /* namespace android */

#endif /* HAL_PIPELINE_STAGES_STATS_PROCESS_STAGE_H */
