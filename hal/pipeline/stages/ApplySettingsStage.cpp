#undef  LOG_TAG
#define LOG_TAG "Cam-ApplySettings"

#include "ApplySettingsStage.h"

#include <log/log.h>

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
#ifndef V4L2_CID_GROUP_HOLD
#define V4L2_CID_GROUP_HOLD   (V4L2_CTRL_CLASS_CAMERA | 0x2003)
#endif

/* Brackets a set of sensor writes with the driver's group hold, so the
 * sensor latches them on the same frame. Without it exposure and gain
 * of one decision can land a frame apart. The driver enters the hold
 * on the first register write after GROUP_HOLD=1 and releases it,
 * committing all held registers at once, on GROUP_HOLD=0. */
class GroupHold {
public:
    GroupHold(V4l2Device *d, bool on) : dev(d), active(on) {
        if (active) dev->setControl(V4L2_CID_GROUP_HOLD, 1);
    }
    ~GroupHold() {
        if (active) dev->setControl(V4L2_CID_GROUP_HOLD, 0);
    }
private:
    V4l2Device *dev;
    bool        active;
};

void writeAeTriple(V4l2Device *dev, const ExposureWriteValues &v,
                   bool groupHold) {
    GroupHold hold(dev, groupHold);
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

void writeAeBatch(V4l2Device *dev, const DelayedControls::Batch &batch,
                  bool groupHold) {
    GroupHold hold(dev, groupHold);
    V4l2Controls ctrls;
    if (batch.has[DelayedControls::EXPOSURE])
        ctrls.add(V4L2_CID_EXPOSURE, batch.val[DelayedControls::EXPOSURE]);
    if (batch.has[DelayedControls::GAIN])
        ctrls.add(V4L2_CID_GAIN, batch.val[DelayedControls::GAIN]);
    if (ctrls.count > 0) dev->setControls(ctrls);
}

} /* namespace */

ApplySettingsStage::ApplySettingsStage(const Deps &d)
    : deps(d),
      lastAutoValid(false),
      lastAutoExposureUs(0),
      lastAutoGain(0),
      groupHold(false) {
    if (deps.dev) {
        int32_t mn = 0, mx = 0, df = 0;
        groupHold = deps.dev->queryControl(V4L2_CID_GROUP_HOLD, &mn, &mx, &df)
                 && mx >= 1;
        ALOGI("sensor group hold %s", groupHold ? "supported" : "not supported");
    }
}

void ApplySettingsStage::reset() {
    lastAutoValid      = false;
    lastAutoExposureUs = 0;
    lastAutoGain       = 0;
}

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
            DelayedControls::Batch batch =
                deps.delayedControls->pendingWrite(ctx.request.frameNumber);
            const bool landed = batch.has[DelayedControls::EXPOSURE]
                             || batch.has[DelayedControls::GAIN];
            if (landed || lastAutoValid) {
                /* Fill whatever the IPA did not decide for this
                 * request from the last auto write, so the sensor
                 * stays on the auto operating point either way. */
                if (!batch.has[DelayedControls::EXPOSURE] && lastAutoValid) {
                    batch.has[DelayedControls::EXPOSURE] = true;
                    batch.val[DelayedControls::EXPOSURE] = lastAutoExposureUs;
                }
                if (!batch.has[DelayedControls::GAIN] && lastAutoValid) {
                    batch.has[DelayedControls::GAIN] = true;
                    batch.val[DelayedControls::GAIN] = lastAutoGain;
                }
                writeAeBatch(deps.dev, batch, groupHold);
                if (batch.has[DelayedControls::EXPOSURE])
                    ctx.appliedExposureUs = batch.val[DelayedControls::EXPOSURE];
                if (batch.has[DelayedControls::GAIN])
                    ctx.appliedGain       = batch.val[DelayedControls::GAIN];
                lastAutoExposureUs = ctx.appliedExposureUs;
                lastAutoGain       = ctx.appliedGain;
                lastAutoValid      = true;
                deps.delayedControls->commit(ctx.request.frameNumber, batch);
                appliedFromRing = true;
            }
        }

        if (!appliedFromRing) {
            /* Manual AE, or auto before the first IPA decision of the
             * stage's lifetime: parse the request deterministically
             * and write V4L2 directly. */
            const ExposureWriteValues v =
                AutoExposureController::parseManualSettings(
                    ctx.request.settings, *deps.sensorCfg);
            writeAeTriple(deps.dev, v, groupHold);
            ctx.appliedExposureUs = v.exposureUs;
            ctx.appliedGain       = v.gain;

            /* Record the value physically written at slot
             * request.frameNumber so applyControls() reports what the
             * sensor really got, and a late IPA push cannot overwrite
             * it. The auto path records its own write above. */
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
                deps.delayedControls->commit(ctx.request.frameNumber, published);
            }
        }
    }

    if (deps.af) {
        deps.af->onSettings(ctx.request.settings, ctx.request.frameNumber);
    }
}

} /* namespace android */
