#ifndef HAL_PIPELINE_STAGES_RESULT_DISPATCH_STAGE_H
#define HAL_PIPELINE_STAGES_RESULT_DISPATCH_STAGE_H

#include <hardware/camera3.h>

#include "PipelineStage.h"

namespace android {

class BayerSource;
class AutoFocusController;
class PartialEmitter;
struct SensorConfig;

/* Always-run terminal stage. On the success path: builds result
 * metadata via ResultMetadataBuilder, unlocks any gralloc outputs
 * that stages left locked, emits the final partial via
 * PartialEmitter (carrying buffer pointers), releases the Bayer
 * frame to the source. On the error path: emits notify(ERROR_REQUEST)
 * via the framework callback and does the minimum cleanup required
 * to leave the HAL in a consistent state.
 *
 * `notify` (shutter, error) still goes through the raw
 * camera3_callback_ops_t pointer — partial-emit abstraction covers
 * `process_capture_result` only. */
class ResultDispatchStage : public PipelineStage {
public:
    struct Deps {
        const camera3_callback_ops_t *const *callbackOps;
        PartialEmitter                      *emitter;
        BayerSource                         *bayerSource;
        AutoFocusController                 *af;         /* may be null */
        const SensorConfig                  *sensorCfg;
    };

    explicit ResultDispatchStage(const Deps &deps);

    const char *name() const override { return "ResultDispatch"; }
    void process(PipelineContext &context) override;
    bool alwaysRun() const override { return true; }

private:
    Deps deps;
};

} /* namespace android */

#endif /* HAL_PIPELINE_STAGES_RESULT_DISPATCH_STAGE_H */
