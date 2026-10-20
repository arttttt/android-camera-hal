#ifndef HAL_PIPELINE_STAGES_SHUTTER_NOTIFY_STAGE_H
#define HAL_PIPELINE_STAGES_SHUTTER_NOTIFY_STAGE_H

#include "PipelineStage.h"

namespace mocha::camera { class ResultSink; }

namespace android {

/* Records the shutter timestamp into the frame and tells the result
 * sink. */
class ShutterNotifyStage : public PipelineStage {
public:
    explicit ShutterNotifyStage(mocha::camera::ResultSink *sink);

    const char *name() const override { return "ShutterNotify"; }
    void process(mocha::camera::Frame &context) override;

private:
    mocha::camera::ResultSink *sink;
};

} /* namespace android */

#endif /* HAL_PIPELINE_STAGES_SHUTTER_NOTIFY_STAGE_H */
