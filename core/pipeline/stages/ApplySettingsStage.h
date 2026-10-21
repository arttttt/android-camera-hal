#ifndef HAL_PIPELINE_STAGES_APPLY_SETTINGS_STAGE_H
#define HAL_PIPELINE_STAGES_APPLY_SETTINGS_STAGE_H

#include <stdint.h>
#include "PipelineStage.h"
#include "mochacamera/Controls.h"
#include "sensor/DelayedControls.h"

namespace android {

class  AutoFocusController;
class  V4l2Device;
struct SensorConfig;
class  ViController;

/* Applies per-request exposure + AF settings to the sensor/VCM.
 *
 * AE branch:
 *   AE Off → manual path.
 *     AutoExposureController::parseManualSettings turns the frame's
 *     settings into an (exposureUs, gain, frameLen) triple; the
 *     stage writes V4L2 directly via V4l2Device::setControls. The
 *     applied triple is also committed into DelayedControls at slot
 *     frameNumber so result metadata reporting sees the same
 *     trajectory.
 *   AE On → auto path.
 *     DelayedControls::pendingWrite(frameNumber) yields the
 *     IPA-decided batch; the stage pushes it through V4L2 in one
 *     VIDIOC_S_EXT_CTRLS call. If the ring is empty for this slot —
 *     cold start, or StubIpa — we fall back to the manual path so
 *     the sensor is never left frozen during bring-up.
 *
 * Either way the frame length is written every frame, and the VI
 * frame wait is raised for it before the write reaches the
 * sensor (ViController).
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
        ViController        *viController;     /* may be null            */
        mocha::camera::Rational evStep;        /* Capability::ae.evStep  */
    };

    explicit ApplySettingsStage(const Deps &deps);

    const char *name() const override { return "ApplySettings"; }
    void process(mocha::camera::Frame &context) override;

    /* Forget the last auto write. Called when a session closes: the
     * next one starts from the driver defaults configureStreams
     * writes, not from the exposure the previous session ended on. */
    void reset();

private:
    Deps deps;

    /* Writes one frame's sensor controls: raises the VI frame wait for
     * the frame length first, then FRAME_LENGTH / EXPOSURE / GAIN in
     * one group hold. */
    void write(int32_t frameLen, int32_t exposureUs, int32_t gain);

    /* Last controls the auto path actually wrote, by DelayedControls
     * id. When the IPA's decision for a request has not landed yet,
     * auto mode repeats this instead of falling back to the request's
     * own sensor settings — those are framework template values, ~1x
     * gain, and writing them mid-stream drops a frame to near black.
     * Invalid until the first auto write of the stage's lifetime. */
    bool    lastAutoValid;
    int32_t lastAuto[DelayedControls::COUNT];

    /* Whether the sensor driver exposes V4L2_CID_GROUP_HOLD. Queried
     * once at construction; writes are bracketed with it when true. */
    bool    groupHold;
};

} /* namespace android */

#endif /* HAL_PIPELINE_STAGES_APPLY_SETTINGS_STAGE_H */
