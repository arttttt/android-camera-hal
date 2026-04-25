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
    fctx.jpegBufferSize = *deps.jpegBufferSize;

    size_t n = ctx.request.outputBuffers.size();
    ctx.outputReleaseFences.assign(n, -1);
    ctx.outputStatuses.assign(n, CAMERA3_BUFFER_STATUS_OK);
    ctx.outputNeedsFinalUnlock.assign(n, true);

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
                                                     &state);
        if (e != NO_ERROR) {
            ALOGE("processOne failed at output %zu for frame %u: %d",
                  i, ctx.request.frameNumber, (int)e);
            for (size_t j = 0; j <= i; ++j) {
                GraphicBufferMapper::get().unlock(*ctx.request.outputBuffers[j].buffer);
            }
            /* Earlier outputs in this request already got QSRIA
             * release-fences we won't be returning to the framework
             * (ResultDispatch's error branch emits notify(ERROR_REQUEST)
             * and skips the per-output plumbing). Close them here so
             * they don't leak as orphan sync_fds. */
            for (size_t j = 0; j < i; ++j) {
                if (ctx.outputReleaseFences[j] >= 0) {
                    ::close(ctx.outputReleaseFences[j]);
                    ctx.outputReleaseFences[j] = -1;
                }
            }
            ctx.outputNeedsFinalUnlock.assign(n, false);
            ctx.errorCode = (int)e;
            return;
        }
        ctx.outputNeedsFinalUnlock[i] = state.needsFinalUnlock;
        ctx.outputReleaseFences[i]    = state.releaseFd;
        if (state.submitSyncFd >= 0)
            ctx.pendingFenceFds.push_back(state.submitSyncFd);
    }
}

} /* namespace android */
