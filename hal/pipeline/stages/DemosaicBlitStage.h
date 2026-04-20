#ifndef HAL_PIPELINE_STAGES_DEMOSAIC_BLIT_STAGE_H
#define HAL_PIPELINE_STAGES_DEMOSAIC_BLIT_STAGE_H

#include <stddef.h>

#include "PipelineStage.h"

namespace android {

class BufferProcessor;
class BayerSource;
class AutoFocusController;

/* Runs BufferProcessor::processOne for every output buffer in the
 * request. Populates context.outputReleaseFences / outputStatuses /
 * outputNeedsFinalUnlock. Hands an RGBA preview pointer (when the
 * per-frame AF sweep forces a SW_READ lock) to AutoFocusController
 * after the loop. */
class DemosaicBlitStage : public PipelineStage {
public:
    struct Deps {
        BufferProcessor      *bufferProcessor;
        BayerSource          *bayerSource;   /* for resolution() */
        AutoFocusController  *af;            /* may be null */
        const size_t         *jpegBufferSize;
    };

    explicit DemosaicBlitStage(const Deps &deps);

    const char *name() const override { return "DemosaicBlit"; }
    void process(PipelineContext &context) override;

private:
    Deps deps;
};

} /* namespace android */

#endif /* HAL_PIPELINE_STAGES_DEMOSAIC_BLIT_STAGE_H */
