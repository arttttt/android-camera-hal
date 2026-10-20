#include "DemosaicBlitStage.h"

#include <stddef.h>
#include <stdint.h>
#include <unistd.h>

#include <utils/Errors.h>
#include <utils/Log.h>
#include <ui/GraphicBufferMapper.h>
#include <hardware/camera3.h>

#include "Frame.h"
#include "BufferProcessor.h"
#include "BayerSource.h"
#include "IspPipeline.h"
#include "DeviceErrorReporter.h"

#undef  LOG_TAG
#define LOG_TAG "Cam-DemosaicBlitStage"

namespace android {

DemosaicBlitStage::DemosaicBlitStage(const Deps &d) : deps(d) {}

void DemosaicBlitStage::reportIfDeviceLost() {
    if (deps.deviceError && deps.isp->deviceLost())
        deps.deviceError->report("ISP GPU device lost");
}

UniqueFd DemosaicBlitStage::process(mocha::camera::Frame &ctx) {
    Resolution res = deps.bayerSource->resolution();
    const V4l2Device::VBuffer *bayer = ctx.bayer.buffer();

    BufferProcessor::FrameContext fctx;
    fctx.frameBuf       = bayer->buf;
    fctx.frameSlotIdx   = (bayer->buf == nullptr) ? bayer->index : -1;
    fctx.pixFmt         = bayer->pixFmt;
    fctx.resW           = res.width;
    fctx.resH           = res.height;
    fctx.cropX          = ctx.cropX;
    fctx.cropY          = ctx.cropY;
    fctx.cropW          = ctx.cropW;
    fctx.cropH          = ctx.cropH;

    size_t n = ctx.request.outputBuffers.size();
    ctx.outputReleaseFences.assign(n, -1);
    ctx.outputStatuses.assign(n, CAMERA3_BUFFER_STATUS_OK);
    ctx.outputJpegSnapshots.assign(n, JpegSnapshot{nullptr, 0, 0, 0, -1});
    ctx.jpegRing = deps.isp;

    /* Open the ISP recording for this frame — demosaic gets recorded once;
     * each blitTo* call inside the loop appends a per-output operation to
     * the same command buffer; endFrame submits the lot. */
    if (!deps.isp->beginFrame(res.width, res.height, fctx.pixFmt, fctx.frameSlotIdx)) {
        ALOGE("beginFrame failed for frame %u", ctx.request.frameNumber);
        ctx.status.fail(NO_INIT);
        reportIfDeviceLost();
        return UniqueFd();
    }

    for (size_t i = 0; i < n; ++i) {
        CaptureRequest::Buffer &outBuf = ctx.request.outputBuffers[i];

        camera3_stream_buffer sb;
        sb.stream        = outBuf.stream;
        sb.buffer        = outBuf.buffer;
        sb.status        = CAMERA3_BUFFER_STATUS_OK;
        sb.acquire_fence = outBuf.acquireFence.release();
        sb.release_fence = -1;

        /* processOne owns the acquire fence from here, whatever it
         * returns: it waited on it, imported it, or closed it. */
        status_t e = deps.bufferProcessor->processOne(sb, fctx, ctx.request.settings,
                                                     ctx.request.frameNumber,
                                                     &ctx.outputReleaseFences[i],
                                                     &ctx.outputJpegSnapshots[i]);
        if (e != NO_ERROR) {
            /* One output that cannot be produced does not cost the
             * frame its other outputs: nothing was recorded for this
             * one, so the frame goes on and ResultDispatch reports the
             * buffer with ERROR_BUFFER. */
            ALOGE("processOne failed at output %zu for frame %u: %d",
                  i, ctx.request.frameNumber, (int)e);
            ctx.outputStatuses[i] = CAMERA3_BUFFER_STATUS_ERROR;
        }
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
        ctx.status.fail(NO_INIT);
        reportIfDeviceLost();
        return UniqueFd();
    }

    return UniqueFd(submitFenceFd);
}

} /* namespace android */
