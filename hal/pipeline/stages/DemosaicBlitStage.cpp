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
    ctx.outputNeedsFinalUnlock.assign(n, true);
    ctx.outputDeferredNvBlit.assign(n, false);
    ctx.outputJpegSnapshots.assign(n, JpegSnapshot{nullptr, 0, 0, 0, -1});
    ctx.jpegRing = deps.isp;

    /* Open the ISP recording for this frame — demosaic gets recorded once;
     * each blitTo* call inside the loop appends a per-output operation to
     * the same command buffer; endFrame submits the lot. */
    if (!deps.isp->beginFrame(res.width, res.height, fctx.pixFmt, fctx.frameSlotIdx)) {
        ALOGE("beginFrame failed for frame %u", ctx.request.frameNumber);
        ctx.status.fail(NO_INIT);
        reportIfDeviceLost();
        ctx.outputNeedsFinalUnlock.assign(n, false);
        return UniqueFd();
    }

    /* Two-pass:
     *  1. Record Vulkan ops for every output (RGBA / BLOB / YUV-fallback)
     *     and remember acquire fences for the HW VIC bridge path. NvBlit
     *     can't fire here because the Vulkan submit fence doesn't exist
     *     yet — we need it to chain "scratch written" → "NvBlit reads".
     *  2. endFrame() submits Vulkan + exports the submit fence.
     *  3. For each output the processOne pass deferred, run NvBlit with
     *     a dup of the submit fence as src wait + the framework
     *     acquire_fence as dst wait. NvBlit returns a release sync_fd
     *     that becomes the output's release_fence to the framework.
     *
     * sbs[] keeps acquire_fence ownership alive across the endFrame
     * boundary — the local sb in the loop would otherwise have dropped
     * out of scope before we get to call processYuvNvBlit. */
    std::vector<camera3_stream_buffer> sbs(n);
    std::vector<bool> deferredYuv(n, false);

    for (size_t i = 0; i < n; ++i) {
        CaptureRequest::Buffer &outBuf = ctx.request.outputBuffers[i];

        sbs[i].stream        = outBuf.stream;
        sbs[i].buffer        = outBuf.buffer;
        sbs[i].status        = CAMERA3_BUFFER_STATUS_OK;
        sbs[i].acquire_fence = outBuf.acquireFence.release();
        sbs[i].release_fence = -1;

        BufferProcessor::OutputState state;
        status_t e = deps.bufferProcessor->processOne(sbs[i], fctx, ctx.request.settings,
                                                     ctx.request.frameNumber,
                                                     &state,
                                                     &ctx.outputReleaseFences[i],
                                                     &ctx.outputJpegSnapshots[i]);
        /* processOne owns the acquire fence from here, whatever it
         * returns: it waited on it, imported it, or closed it — unless
         * it deferred the output to the VIC bridge, which takes the
         * fence in pass 3. */
        if (e != NO_ERROR || !state.deferredNvBlit)
            sbs[i].acquire_fence = -1;
        if (e != NO_ERROR) {
            /* One output that cannot be produced does not cost the
             * frame its other outputs: nothing was recorded for this
             * one, so the frame goes on and ResultDispatch reports the
             * buffer with ERROR_BUFFER. */
            ALOGE("processOne failed at output %zu for frame %u: %d",
                  i, ctx.request.frameNumber, (int)e);
            ctx.outputStatuses[i]         = CAMERA3_BUFFER_STATUS_ERROR;
            ctx.outputNeedsFinalUnlock[i] = false;
            continue;
        }
        ctx.outputNeedsFinalUnlock[i] = state.needsFinalUnlock;
        ctx.outputDeferredNvBlit[i]   = state.deferredNvBlit;
        deferredYuv[i]                = state.deferredNvBlit;
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
        ctx.status.fail(NO_INIT);
        reportIfDeviceLost();
        return UniqueFd();
    }

    /* Pass 3 — fire HW VIC blits for the outputs processOne deferred.
     * processYuvNvBlit owns the dup of submitFenceFd and sbs[i].acquire_fence
     * on success (NvBlit consumes them); on failure it closes them
     * itself and returns NO_INIT, in which case we just mark the output
     * as error. The original submitFenceFd goes to the demosaic engine,
     * which waits on it before the frame moves on. */
    for (size_t i = 0; i < n; ++i) {
        if (!deferredYuv[i]) continue;
        status_t e = deps.bufferProcessor->processYuvNvBlit(
            sbs[i], fctx, ctx.request.frameNumber,
            submitFenceFd, &ctx.outputReleaseFences[i]);
        sbs[i].acquire_fence = -1;   /* processYuvNvBlit took it over */
        if (e != NO_ERROR) {
            ctx.outputStatuses[i] = CAMERA3_BUFFER_STATUS_ERROR;
        }
    }

    return UniqueFd(submitFenceFd);
}

} /* namespace android */
