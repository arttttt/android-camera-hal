#undef  LOG_TAG
#define LOG_TAG "Cam-ApplySettings"

#include "ApplySettingsStage.h"

#include <optional>

#include <log/log.h>

#include <linux/videodev2.h>


#include "Frame.h"
#include "3a/AutoExposureController.h"
#include "3a/AutoFocusController.h"
#include "sensor/DelayedControls.h"
#include "sensor/ExposureTarget.h"
#include "sensor/SensorConfig.h"
#include "ViController.h"
#include "V4l2Controls.h"
#include "V4l2Device.h"

namespace android {

namespace {

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

} /* namespace */

ApplySettingsStage::ApplySettingsStage(const Deps &d)
    : deps(d),
      groupHold(false) {
    if (deps.dev) {
        int32_t mn = 0, mx = 0, df = 0;
        groupHold = deps.dev->queryControl(V4L2_CID_GROUP_HOLD, &mn, &mx, &df)
                 && mx >= 1;
        ALOGI("sensor group hold %s", groupHold ? "supported" : "not supported");
    }
}

void ApplySettingsStage::write(int32_t frameLen, int32_t lineLength,
                               int32_t exposureUs, int32_t gain) {
    if (deps.viController)
        deps.viController->beforeFrame(
            deps.sensorCfg->frameDurationNs(frameLen, lineLength));

    GroupHold hold(deps.dev, groupHold);
    /* Single VIDIOC_S_EXT_CTRLS — frame_length must reach the sensor
     * before / together with the exposure that needs it, otherwise
     * the driver clamps the exposure into the old (smaller) frame
     * and the user sees nothing change. */
    V4l2Controls ctrls;
    ctrls.add(V4L2_CID_FRAME_LENGTH, frameLen);
    /* The line before the exposure counted in it: the driver converts
     * µs to lines of the length in effect, and setControls sends the
     * camera class (frame and line length) before the user class
     * (exposure, gain). */
    if (deps.sensorCfg->lineLengthMax > 0)
        ctrls.add(V4L2_CID_LINE_LENGTH, lineLength);
    ctrls.add(V4L2_CID_EXPOSURE,     exposureUs);
    ctrls.add(V4L2_CID_GAIN,         gain);
    deps.dev->setControls(ctrls);
}

void ApplySettingsStage::process(mocha::camera::Frame &ctx) {
    if (deps.dev && deps.sensorCfg) {
        bool appliedFromTarget = false;
        const std::optional<DelayedControls::Batch> target =
            (ctx.settings.ae.mode != mocha::camera::AeMode::Off &&
             deps.exposureTarget)
                ? deps.exposureTarget->latest() : std::nullopt;
        if (target) {
            /* Auto AE: the IPA's latest decision. Before its first
             * there is none, and the request falls through to the
             * manual path, which keeps the sensor on a valid exposure
             * rather than frozen. */
            DelayedControls::Batch batch = *target;
            /* What the IPA has never decided comes from the defaults
             * configure wrote. */
            const DelayedControls::Batch defaults =
                deps.sensorCfg->configureDefaults();
            for (int id = 0; id < DelayedControls::COUNT; ++id) {
                if (batch.has[id]) continue;
                batch.has[id] = true;
                batch.val[id] = defaults.val[id];
            }
            write(batch.val[DelayedControls::FRAME_LENGTH],
                  batch.val[DelayedControls::LINE_LENGTH],
                  batch.val[DelayedControls::EXPOSURE],
                  batch.val[DelayedControls::GAIN]);
            if (deps.delayedControls)
                deps.delayedControls->commit(ctx.frameNumber, batch);
            appliedFromTarget = true;
        }

        if (!appliedFromTarget) {
            /* Manual AE, or auto before the IPA's first decision of
             * the session: parse the request deterministically
             * and write V4L2 directly. */
            const ExposureWriteValues v =
                AutoExposureController::parseManualSettings(
                    ctx.settings, *deps.sensorCfg, deps.evStep);
            write(v.frameLen, v.lineLength, v.exposureUs, v.gain);

            /* Record the value physically written at slot
             * frameNumber so applyControls() reports what the
             * sensor really got. The auto path records its own write
             * above. */
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
                published.has[DelayedControls::FRAME_LENGTH] = true;
                published.val[DelayedControls::FRAME_LENGTH] = v.frameLen;
                published.has[DelayedControls::LINE_LENGTH]  = true;
                published.val[DelayedControls::LINE_LENGTH]  = v.lineLength;
                deps.delayedControls->commit(ctx.frameNumber, published);
            }
        }
    }

    /* What is in effect on the frame this request takes: written a
     * control delay before, or what configure wrote. */
    if (deps.delayedControls && deps.sensorCfg)
        ctx.inEffect = deps.delayedControls->applyControls(
            ctx.frameNumber, deps.sensorCfg->configureDefaults());

    if (deps.af) {
        deps.af->onSettings(ctx.settings, ctx.frameNumber);
    }
}

} /* namespace android */
