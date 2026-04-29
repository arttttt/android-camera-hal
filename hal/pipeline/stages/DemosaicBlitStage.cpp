#include "DemosaicBlitStage.h"

#include <stddef.h>
#include <stdint.h>
#include <unistd.h>

#include <utils/Errors.h>
#include <utils/Log.h>
#include <ui/GraphicBufferMapper.h>
#include <hardware/camera3.h>

#include "PipelineContext.h"
#include "BufferProcessor.h"
#include "BayerSource.h"
#include "IspPipeline.h"

#define LOG_TAG "Cam-DemosaicBlitStage"

namespace android {

DemosaicBlitStage::DemosaicBlitStage(const Deps &d) : deps(d) {}

void DemosaicBlitStage::process(PipelineContext &ctx) {
    Resolution res = deps.bayerSource->resolution();

    BufferProcessor::FrameContext fctx;
    fctx.frameBuf       = ctx.bayerFrame->buf;
    fctx.frameSlotIdx   = (ctx.bayerFrame->buf == nullptr) ? ctx.bayerFrame->index : -1;
    fctx.pixFmt         = ctx.bayerFrame->pixFmt;
    fctx.resW           = res.width;
    fctx.resH           = res.height;
    fctx.cropX          = ctx.cropX;
    fctx.cropY          = ctx.cropY;
    fctx.cropW          = ctx.cropW;
    fctx.cropH          = ctx.cropH;

    size_t n = ctx.request.outputBuffers.size();
    ctx.outputReleaseFences.assign(n, -1);
    ctx.outputStatuses.assign(n, CAMERA3_BUFFER_STATUS_OK);
    ctx.outputNeedsFinalUnlock.assign(n, true);
    ctx.outputJpegSnapshots.assign(n, JpegSnapshot{nullptr, 0, 0, 0, -1});

    /* Open the ISP recording for this frame — demosaic gets recorded once;
     * each blitTo* call inside the loop appends a per-output operation to
     * the same command buffer; endFrame submits the lot. */
    if (!deps.isp->beginFrame(res.width, res.height, fctx.pixFmt, fctx.frameSlotIdx)) {
        ALOGE("beginFrame failed for frame %u", ctx.request.frameNumber);
        ctx.errorCode = NO_INIT;
        ctx.outputNeedsFinalUnlock.assign(n, false);
        return;
    }

    for (size_t i = 0; i < n; ++i) {
        CaptureRequest::Buffer &outBuf = ctx.request.outputBuffers[i];

        camera3_stream_buffer sb;
        sb.stream        = outBuf.stream;
        sb.buffer        = outBuf.buffer;
        sb.status        = CAMERA3_BUFFER_STATUS_OK;
        sb.acquire_fence = outBuf.acquireFence.release();
        sb.release_fence = -1;

        BufferProcessor::OutputState state;
        status_t e = deps.bufferProcessor->processOne(sb, fctx, ctx.request.settings,
                                                     ctx.request.frameNumber,
                                                     &state,
                                                     &ctx.outputReleaseFences[i],
                                                     &ctx.outputJpegSnapshots[i]);
        if (e != NO_ERROR) {
            ALOGE("processOne failed at output %zu for frame %u: %d",
                  i, ctx.request.frameNumber, (int)e);
            for (size_t j = 0; j <= i; ++j) {
                GraphicBufferMapper::get().unlock(*ctx.request.outputBuffers[j].buffer);
            }
            for (size_t j = 0; j < i; ++j) {
                if (ctx.outputReleaseFences[j] >= 0) {
                    ::close(ctx.outputReleaseFences[j]);
                    ctx.outputReleaseFences[j] = -1;
                }
            }
            ctx.outputNeedsFinalUnlock.assign(n, false);
            ctx.errorCode = (int)e;
            /* Submit the partial recording anyway so slot fences end up in a
             * consistent state for the next frame; release fences for any
             * already-recorded blits get dropped above. */
            int submitFd = -1;
            (void)deps.isp->endFrame(&submitFd);
            if (submitFd >= 0) ::close(submitFd);
            return;
        }
        ctx.outputNeedsFinalUnlock[i] = state.needsFinalUnlock;
    }

    int submitFenceFd = -1;
    if (!deps.isp->endFrame(&submitFenceFd)) {
        ALOGE("endFrame failed for frame %u", ctx.request.frameNumber);
        for (size_t j = 0; j < n; ++j) {
            if (ctx.outputReleaseFences[j] >= 0) {
                ::close(ctx.outputReleaseFences[j]);
                ctx.outputReleaseFences[j] = -1;
            }
        }
        ctx.outputNeedsFinalUnlock.assign(n, false);
        ctx.errorCode = NO_INIT;
        return;
    }
    if (submitFenceFd >= 0)
        ctx.pendingFenceFds.push_back(submitFenceFd);
}

} /* namespace android */
