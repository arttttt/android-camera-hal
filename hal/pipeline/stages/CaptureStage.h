#ifndef HAL_PIPELINE_STAGES_CAPTURE_STAGE_H
#define HAL_PIPELINE_STAGES_CAPTURE_STAGE_H

#include "PipelineStage.h"

namespace android {

class V4l2Device;
class IspPipeline;
class AutoFocusController;

/* Synchronously acquires a Bayer frame from V4L2 and parses the
 * per-request crop region (digital zoom). Will migrate to a
 * CaptureThread in a later change — at that point this stage is
 * rehomed onto that thread's Pipeline while keeping the same body. */
class CaptureStage : public PipelineStage {
public:
    struct Deps {
        V4l2Device           *dev;
        IspPipeline          *isp;
        AutoFocusController **af;   /* pointer-to-field; may be null per config */
    };

    explicit CaptureStage(const Deps &deps);

    const char *name() const override { return "Capture"; }
    void process(PipelineContext &context) override;

private:
    Deps deps;
};

} /* namespace android */

#endif /* HAL_PIPELINE_STAGES_CAPTURE_STAGE_H */
