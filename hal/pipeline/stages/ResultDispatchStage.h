#ifndef HAL_PIPELINE_STAGES_RESULT_DISPATCH_STAGE_H
#define HAL_PIPELINE_STAGES_RESULT_DISPATCH_STAGE_H

#include "PipelineStage.h"

namespace mocha::camera { class ResultSink; }

namespace android {

class AutoFocusController;
struct SensorConfig;
class DelayedControls;

/* Always-run terminal stage: the frame's final result, to the result
 * sink. A completed frame goes out with its 3A report (describe3A) and
 * every buffer -- an output that failed goes in error, the rest with
 * their release fences. A failed or cancelled frame goes out as a
 * failed request: every buffer in error, its release fence the
 * acquire fence it came with if the HAL still holds it, else whatever
 * fence the GPU was told to signal for it. */
class ResultDispatchStage : public PipelineStage {
public:
    struct Deps {
        mocha::camera::ResultSink *sink;
        AutoFocusController       *af;               /* may be null */
        const SensorConfig        *sensorCfg;
        const DelayedControls     *delayedControls;  /* may be null */
    };

    explicit ResultDispatchStage(const Deps &deps);

    const char *name() const override { return "ResultDispatch"; }
    void process(mocha::camera::Frame &frame) override;
    bool alwaysRun() const override { return true; }

private:
    Deps deps;
};

} /* namespace android */

#endif /* HAL_PIPELINE_STAGES_RESULT_DISPATCH_STAGE_H */
