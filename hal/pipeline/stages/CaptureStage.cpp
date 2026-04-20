#include "CaptureStage.h"

#include <utils/Errors.h>
#include <utils/Log.h>
#include <utils/Timers.h>

#include "PipelineContext.h"
#include "V4l2Device.h"
#include "IspPipeline.h"
#include "3a/AutoFocusController.h"

#define LOG_TAG "Cam-CaptureStage"

namespace android {

CaptureStage::CaptureStage(const Deps &d) : deps(d) {}

void CaptureStage::process(PipelineContext &ctx) {
    /* Drain the previous frame's GPU work before V4L2 can reuse the
     * input slot. Synchronous here; migrates to a fence-fd poll-set
     * on the CaptureThread / PipelineThread in a later change. */
    deps.isp->waitForPreviousFrame();

    const V4l2Device::VBuffer *frame = deps.dev->readLock();
    ctx.tBayerDq = systemTime();

    if (!frame) {
        ALOGW("readLock returned null for frame %u", ctx.request.frameNumber);
        ctx.errorCode = NOT_ENOUGH_DATA;
        return;
    }
    ctx.bayerFrame = frame;

    AutoFocusController *af = *deps.af;
    if (af) af->onFrameStart();

    Resolution res = deps.dev->resolution();

    int cropX = 0;
    int cropY = 0;
    int cropW = res.width;
    int cropH = res.height;

    if (ctx.request.settings.exists(ANDROID_SCALER_CROP_REGION)) {
        camera_metadata_entry_t entry =
            ctx.request.settings.find(ANDROID_SCALER_CROP_REGION);
        const int32_t *crop = entry.data.i32;
        Resolution sensor = deps.dev->sensorResolution();
        cropX = crop[0] * (int)res.width  / (int)sensor.width;
        cropY = crop[1] * (int)res.height / (int)sensor.height;
        cropW = crop[2] * (int)res.width  / (int)sensor.width;
        cropH = crop[3] * (int)res.height / (int)sensor.height;
        if (cropX < 0) cropX = 0;
        if (cropY < 0) cropY = 0;
        if (cropW < 16) cropW = 16;
        if (cropH < 16) cropH = 16;
        if (cropX + cropW > (int)res.width)  cropW = (int)res.width  - cropX;
        if (cropY + cropH > (int)res.height) cropH = (int)res.height - cropY;
    }

    ctx.cropX = cropX;
    ctx.cropY = cropY;
    ctx.cropW = cropW;
    ctx.cropH = cropH;
}

} /* namespace android */
