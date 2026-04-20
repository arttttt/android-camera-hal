#ifndef HAL_PIPELINE_STAGES_SHUTTER_NOTIFY_STAGE_H
#define HAL_PIPELINE_STAGES_SHUTTER_NOTIFY_STAGE_H

#include <hardware/camera3.h>

#include "PipelineStage.h"

namespace android {

/* Records the shutter timestamp into the context and emits
 * CAMERA3_MSG_SHUTTER to the framework. */
class ShutterNotifyStage : public PipelineStage {
public:
    explicit ShutterNotifyStage(const camera3_callback_ops_t *const *callbackOps);

    const char *name() const override { return "ShutterNotify"; }
    void process(PipelineContext &context) override;

private:
    const camera3_callback_ops_t *const *callbackOps;
};

} /* namespace android */

#endif /* HAL_PIPELINE_STAGES_SHUTTER_NOTIFY_STAGE_H */
