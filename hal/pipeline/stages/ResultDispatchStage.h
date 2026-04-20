#ifndef HAL_PIPELINE_STAGES_RESULT_DISPATCH_STAGE_H
#define HAL_PIPELINE_STAGES_RESULT_DISPATCH_STAGE_H

#include <hardware/camera3.h>
#include <camera/CameraMetadata.h>

#include "PipelineStage.h"

namespace android {

class V4l2Device;
class AutoFocusController;
struct SensorConfig;

/* Always-run terminal stage. On the success path: builds result
 * metadata via ResultMetadataBuilder, unlocks any gralloc outputs
 * that stages left locked, sends process_capture_result, releases
 * the V4L2 bayer frame, updates the last-settings cache. On the
 * error path: emits notify(ERROR_REQUEST) and does the minimum
 * cleanup required to leave the HAL in a consistent state. */
class ResultDispatchStage : public PipelineStage {
public:
    struct Deps {
        const camera3_callback_ops_t *const *callbackOps;
        V4l2Device                          *dev;
        AutoFocusController                **af;
        const SensorConfig                  *sensorCfg;
        CameraMetadata                      *lastRequestSettings;
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
