#ifndef HAL_PIPELINE_STAGES_CAPTURE_STAGE_H
#define HAL_PIPELINE_STAGES_CAPTURE_STAGE_H

#include "PipelineStage.h"

namespace android {

class BayerSource;
class IspPipeline;
class AutoFocusController;

/* Acquires a Bayer frame from the BayerSource and parses the
 * per-request crop region (digital zoom). The source's own thread
 * handles V4L2 poll + DQBUF + drain-to-latest — this stage just waits
 * for the freshest available frame. */
class CaptureStage : public PipelineStage {
public:
    struct Deps {
        BayerSource         *bayerSource;
        IspPipeline         *isp;
        AutoFocusController *af;          /* may be null */
    };

    explicit CaptureStage(const Deps &deps);

    const char *name() const override { return "Capture"; }
    void process(PipelineContext &context) override;

private:
    Deps deps;
};

} /* namespace android */

#endif /* HAL_PIPELINE_STAGES_CAPTURE_STAGE_H */
