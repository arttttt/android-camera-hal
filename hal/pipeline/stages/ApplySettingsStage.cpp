#include "ApplySettingsStage.h"

#include <linux/videodev2.h>

#include <system/camera_metadata.h>

#include "PipelineContext.h"
#include "3a/AutoExposureController.h"
#include "3a/AutoFocusController.h"
#include "sensor/DelayedControls.h"
#include "sensor/SensorConfig.h"
#include "V4l2Controls.h"
#include "V4l2Device.h"

namespace android {

namespace {

/* Tegra-specific frame_length CID — not in the standard V4L2 headers.
 * Same magic the IMX179 / OV5693 kernel drivers expose; matches the
 * value used elsewhere in this HAL. */
#ifndef V4L2_CID_FRAME_LENGTH
#define V4L2_CID_FRAME_LENGTH (V4L2_CTRL_CLASS_CAMERA | 0x2000)
#endif

void writeAeTriple(V4l2Device *dev, const ExposureWriteValues &v) {
    /* Single VIDIOC_S_EXT_CTRLS — frame_length must reach the sensor
     * before / together with the exposure that needs it, otherwise
     * the driver clamps the exposure into the old (smaller) frame
     * and the user sees nothing change. */
    V4l2Controls ctrls;
    ctrls.add(V4L2_CID_FRAME_LENGTH, v.frameLen);
    ctrls.add(V4L2_CID_EXPOSURE,     v.exposureUs);
    ctrls.add(V4L2_CID_GAIN,         v.gain);
    dev->setControls(ctrls);
}

void writeAeBatch(V4l2Device *dev, const DelayedControls::Batch &batch) {
    V4l2Controls ctrls;
    if (batch.has[DelayedControls::EXPOSURE])
        ctrls.add(V4L2_CID_EXPOSURE, batch.val[DelayedControls::EXPOSURE]);
    if (batch.has[DelayedControls::GAIN])
        ctrls.add(V4L2_CID_GAIN, batch.val[DelayedControls::GAIN]);
    if (ctrls.count > 0) dev->setControls(ctrls);
}

} /* namespace */

ApplySettingsStage::ApplySettingsStage(const Deps &d) : deps(d) {}

void ApplySettingsStage::process(PipelineContext &ctx) {
    if (deps.sensorCfg) {
        ctx.appliedExposureUs = deps.sensorCfg->exposureDefault;
        ctx.appliedGain       = deps.sensorCfg->gainDefault;
    }

    if (deps.dev && deps.sensorCfg) {
        uint8_t aeMode = ANDROID_CONTROL_AE_MODE_ON;
        if (ctx.request.settings.exists(ANDROID_CONTROL_AE_MODE)) {
            aeMode = *ctx.request.settings.find(ANDROID_CONTROL_AE_MODE).data.u8;
        }

        bool appliedFromRing = false;
        if (aeMode != ANDROID_CONTROL_AE_MODE_OFF && deps.delayedControls) {
            /* Auto AE — consume the IPA's decision if it has landed
             * for this frame. The ring is empty on cold start and
             * whenever the IPA returns empty batches; fall through
             * to the manual path so the sensor keeps a valid
             * exposure / gain rather than freezing. */
            const DelayedControls::Batch batch =
                deps.delayedControls->pendingWrite(ctx.request.frameNumber);
            if (batch.has[DelayedControls::EXPOSURE]
             || batch.has[DelayedControls::GAIN]) {
                writeAeBatch(deps.dev, batch);
                if (batch.has[DelayedControls::EXPOSURE])
                    ctx.appliedExposureUs = batch.val[DelayedControls::EXPOSURE];
                if (batch.has[DelayedControls::GAIN])
                    ctx.appliedGain       = batch.val[DelayedControls::GAIN];
                appliedFromRing = true;
            }
        }

        if (!appliedFromRing) {
            /* Manual AE, or auto-with-no-IPA-push: parse the request
             * deterministically and write V4L2 directly. */
            const ExposureWriteValues v =
                AutoExposureController::parseManualSettings(
                    ctx.request.settings, *deps.sensorCfg);
            writeAeTriple(deps.dev, v);
            ctx.appliedExposureUs = v.exposureUs;
            ctx.appliedGain       = v.gain;

            /* Publish the value physically written at slot
             * request.frameNumber so ResultMetadataBuilder's
             * applyControls(frame + delay) query returns the same
             * numbers we just wrote. Skipped when the auto path
             * already applied from the ring — IPA's push is already
             * there. */
            if (deps.delayedControls) {
                DelayedControls::Batch published;
                for (int i = 0; i < DelayedControls::COUNT; ++i) {
                    published.has[i] = false;
                    published.val[i] = 0;
                }
                published.has[DelayedControls::EXPOSURE] = true;
                published.val[DelayedControls::EXPOSURE] = v.exposureUs;
                published.has[DelayedControls::GAIN]     = true;
                published.val[DelayedControls::GAIN]     = v.gain;
                deps.delayedControls->push(ctx.request.frameNumber, published);
            }
        }
    }

    if (deps.af) {
        deps.af->onSettings(ctx.request.settings, ctx.request.frameNumber);
    }
}

} /* namespace android */
