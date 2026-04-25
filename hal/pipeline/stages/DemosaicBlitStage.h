#ifndef HAL_PIPELINE_STAGES_DEMOSAIC_BLIT_STAGE_H
#define HAL_PIPELINE_STAGES_DEMOSAIC_BLIT_STAGE_H

#include <stddef.h>

#include "PipelineStage.h"

namespace android {

class BufferProcessor;
class BayerSource;
class IspPipeline;

/* Opens the ISP recording for the frame, runs BufferProcessor::processOne
 * for every output buffer (each appending a per-output blit / encode to
 * the open recording), then closes the recording with endFrame to kick off
 * a single GPU submit. Populates ctx.outputReleaseFences /
 * outputStatuses / outputNeedsFinalUnlock and pushes the submit-completion
 * sync_fd into ctx.pendingFenceFds for PipelineThread's fence-reap. */
class DemosaicBlitStage : public PipelineStage {
public:
    struct Deps {
        BufferProcessor *bufferProcessor;
        BayerSource     *bayerSource;   /* for resolution() */
        IspPipeline     *isp;
        const size_t    *jpegBufferSize;
    };

    explicit DemosaicBlitStage(const Deps &deps);

    const char *name() const override { return "DemosaicBlit"; }
    void process(PipelineContext &context) override;

private:
    Deps deps;
};

} /* namespace android */

#endif /* HAL_PIPELINE_STAGES_DEMOSAIC_BLIT_STAGE_H */
