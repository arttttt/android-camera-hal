#ifndef HAL_PIPELINE_STAGES_APPLY_SETTINGS_STAGE_H
#define HAL_PIPELINE_STAGES_APPLY_SETTINGS_STAGE_H

#include <stdint.h>
#include "PipelineStage.h"

namespace android {

class  AutoFocusController;
class  DelayedControls;
class  V4l2Device;
struct SensorConfig;

/* Applies per-request exposure + AF settings to the sensor/VCM.
 *
 * AE branch:
 *   ANDROID_CONTROL_AE_MODE == OFF  → manual path.
 *     AutoExposureController::parseManualSettings turns the request
 *     metadata into an (exposureUs, gain, frameLen) triple; the
 *     stage writes V4L2 directly via V4l2Device::setControls. The
 *     applied (exposure, gain) pair is also published into
 *     DelayedControls::push at slot request.frameNumber so result
 *     metadata reporting sees the same trajectory.
 *   ANDROID_CONTROL_AE_MODE != OFF → auto path.
 *     DelayedControls::pendingWrite(request.frameNumber) yields the
 *     IPA-decided batch; the stage pushes it through V4L2 in one
 *     VIDIOC_S_EXT_CTRLS call. If the ring is empty for this slot —
 *     cold start, or StubIpa — we fall back to the manual path so
 *     the sensor is never left frozen during bring-up.
 *
 * AF is independent of AE mode; the stage forwards request settings
 * to AutoFocusController unchanged. */
class ApplySettingsStage : public PipelineStage {
public:
    struct Deps {
        V4l2Device          *dev;              /* writes sensor V4L2     */
        AutoFocusController *af;               /* may be null            */
        const SensorConfig  *sensorCfg;
        DelayedControls     *delayedControls;  /* may be null            */
    };

    explicit ApplySettingsStage(const Deps &deps);

    const char *name() const override { return "ApplySettings"; }
    void process(PipelineContext &context) override;

private:
    Deps deps;

    /* Last exposure / gain the auto path actually wrote. When the
     * IPA's decision for a request has not landed yet, auto mode
     * repeats this instead of falling back to the request's own
     * sensor settings — those are framework template values, ~1x
     * gain, and writing them mid-stream drops a frame to near black.
     * Invalid until the first auto write of the stage's lifetime. */
    bool    lastAutoValid;
    int32_t lastAutoExposureUs;
    int32_t lastAutoGain;

    /* Whether the sensor driver exposes V4L2_CID_GROUP_HOLD. Queried
     * once at construction; writes are bracketed with it when true. */
    bool    groupHold;
};

} /* namespace android */

#endif /* HAL_PIPELINE_STAGES_APPLY_SETTINGS_STAGE_H */
