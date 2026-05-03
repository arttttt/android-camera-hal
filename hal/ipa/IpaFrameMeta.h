#ifndef HAL_IPA_IPA_FRAME_META_H
#define HAL_IPA_IPA_FRAME_META_H

#include <stdint.h>

#include <system/camera_metadata.h>

#include "IpaStats.h"

namespace android {

/* Per-frame framework control state the IPA needs to decide what to
 * update. Extracted from ctx.request.settings in StatsProcessStage so
 * the IPA stays ignorant of CameraMetadata parsing.
 *
 * Each field defaults to the camera3 spec's "auto" value when the
 * request does not set the corresponding key, so an IPA can treat a
 * missing key as permission to run. */
struct IpaFrameMeta {
    /* ANDROID_CONTROL_AE_MODE — OFF means the framework controls
     * exposure / gain directly; the IPA must not push either. */
    uint8_t aeMode;

    /* ANDROID_CONTROL_AE_LOCK — ON means the IPA must hold the last
     * exposure / gain decision. */
    uint8_t aeLock;

    /* ANDROID_CONTROL_AWB_MODE — OFF means the framework controls
     * colour-correction gains directly; presets (INCANDESCENT etc.)
     * eventually map to pre-tuned WB priors but today every non-AUTO
     * value freezes the gains at their last auto decision. */
    uint8_t awbMode;

    /* ANDROID_CONTROL_AWB_LOCK — ON means the IPA must hold the last
     * WB gains. */
    uint8_t awbLock;

    /* ANDROID_CONTROL_AE_EXPOSURE_COMPENSATION — signed step count in
     * units of ANDROID_CONTROL_AE_COMPENSATION_STEP (1/3 stop in our
     * static metadata). The IPA scales its target by 2^(value/3)
     * before driving the AE controller; the manual-mode equivalent
     * lives in AutoExposureController::parseManualSettings, which
     * applies the same EV scaling against the requested shutter time. */
    int32_t aeExposureCompensation;

    /* ANDROID_COLOR_CORRECTION_GAINS, decoded as the per-channel
     * multipliers the IPA pushes into the demosaic shader. Camera2
     * orders the array as (R, Geven, Godd, B); IpaFrameMeta collapses
     * the two greens into one (they're calibrated equal). manualWbValid
     * gates the path: the field is set only when the request actually
     * carries the key AND awbMode == OFF, so AUTO frames keep the IPA's
     * gray-world output unchanged. */
    bool  manualWbValid;
    float manualWbR;
    float manualWbG;
    float manualWbB;

    /* AF region of interest in IpaStats patch-grid coordinates,
     * half-open on each axis. Defaults to the centre 8×8 patches —
     * same rectangle the AutoFocusController publishes when no
     * tap-to-focus has shifted it.
     *
     * AE / AWB use this rectangle to weight the luma metric and the
     * scene-light gate so metering follows the user's focus tap.
     * Without a tap the rectangle stays centre-default and the
     * pipeline behaves like a plain centre-weighted metric. */
    int focusRoiPyLo;
    int focusRoiPyHi;
    int focusRoiPxLo;
    int focusRoiPxHi;

    IpaFrameMeta()
        : aeMode(ANDROID_CONTROL_AE_MODE_ON),
          aeLock(ANDROID_CONTROL_AE_LOCK_OFF),
          awbMode(ANDROID_CONTROL_AWB_MODE_AUTO),
          awbLock(ANDROID_CONTROL_AWB_LOCK_OFF),
          aeExposureCompensation(0),
          manualWbValid(false),
          manualWbR(1.0f), manualWbG(1.0f), manualWbB(1.0f),
          focusRoiPyLo(IpaStats::FOCUS_ROI_PY_LO),
          focusRoiPyHi(IpaStats::FOCUS_ROI_PY_HI),
          focusRoiPxLo(IpaStats::FOCUS_ROI_PX_LO),
          focusRoiPxHi(IpaStats::FOCUS_ROI_PX_HI) {}
};

} /* namespace android */

#endif /* HAL_IPA_IPA_FRAME_META_H */
