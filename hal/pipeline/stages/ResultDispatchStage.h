#ifndef HAL_PIPELINE_STAGES_RESULT_DISPATCH_STAGE_H
#define HAL_PIPELINE_STAGES_RESULT_DISPATCH_STAGE_H

#include <hardware/camera3.h>

#include "PipelineStage.h"

namespace android {

class AutoFocusController;
class PartialEmitter;
struct SensorConfig;
class DelayedControls;

/* Always-run terminal stage. On the success path: builds result
 * metadata via ResultMetadataBuilder, unlocks any gralloc outputs
 * that stages left locked, emits the final partial via
 * PartialEmitter (carrying buffer pointers). On the error path: emits notify(ERROR_REQUEST)
 * via the framework callback, then returns every output buffer with
 * CAMERA3_BUFFER_STATUS_ERROR and no metadata, as camera3.h requires
 * after ERROR_REQUEST.
 *
 * `notify` (shutter, error) still goes through the raw
 * camera3_callback_ops_t pointer — partial-emit abstraction covers
 * `process_capture_result` only. */
class ResultDispatchStage : public PipelineStage {
public:
    struct Deps {
        const camera3_callback_ops_t *const *callbackOps;
        PartialEmitter                      *emitter;
        AutoFocusController                 *af;         /* may be null */
        const SensorConfig                  *sensorCfg;
        const DelayedControls               *delayedControls; /* may be null */
    };

    explicit ResultDispatchStage(const Deps &deps);

    const char *name() const override { return "ResultDispatch"; }
    void process(mocha::camera::Frame &context) override;
    bool alwaysRun() const override { return true; }

private:
    void returnBuffersInError(mocha::camera::Frame &context);

    Deps deps;
};

} /* namespace android */

#endif /* HAL_PIPELINE_STAGES_RESULT_DISPATCH_STAGE_H */
