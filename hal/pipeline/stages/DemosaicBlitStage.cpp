#include "DemosaicBlitStage.h"

#include <stddef.h>
#include <stdint.h>
#include <unistd.h>

#include <utils/Errors.h>
#include <utils/Log.h>

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

    ctx.jpegRing = deps.isp;

    /* Open the ISP recording for this frame — demosaic gets recorded once;
     * each blitTo* call inside the loop appends a per-output operation to
     * the same command buffer; endFrame submits the lot. */
    if (!deps.isp->beginFrame(res.width, res.height, fctx.pixFmt, fctx.frameSlotIdx)) {
        ALOGE("beginFrame failed for frame %u", ctx.frameNumber);
        ctx.status.fail(NO_INIT);
        reportIfDeviceLost();
        return UniqueFd();
    }

    for (size_t i = 0; i < ctx.outputs.size(); ++i) {
        /* processOne uses the acquire fence up, whatever it returns:
         * it waited on it, imported it, or closed it. An RGBA output's
         * release fence is written by endFrame, through the address the
         * ISP keeps -- the outputs do not move. */
        status_t e = deps.bufferProcessor->processOne(ctx.outputs[i], fctx,
                                                     ctx.frameNumber);
        if (e != NO_ERROR) {
            /* One output that cannot be produced does not cost the
             * frame its other outputs: nothing was recorded for this
             * one, so the frame goes on and its result reports the
             * buffer in error. */
            ALOGE("processOne failed at output %zu for frame %u: %d",
                  i, ctx.frameNumber, (int)e);
            ctx.outputs[i].status = mocha::camera::BufferStatus::Error;
        }
    }

    int submitFenceFd = -1;
    if (!deps.isp->endFrame(&submitFenceFd)) {
        ALOGE("endFrame failed for frame %u", ctx.frameNumber);
        for (mocha::camera::Frame::Output &out : ctx.outputs) {
            if (out.releaseFence >= 0) {
                ::close(out.releaseFence);
                out.releaseFence = -1;
            }
        }
        ctx.status.fail(NO_INIT);
        reportIfDeviceLost();
        return UniqueFd();
    }

    return UniqueFd(submitFenceFd);
}

} /* namespace android */
