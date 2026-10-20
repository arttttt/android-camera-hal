#ifndef HAL_IPA_IPA_FRAME_META_H
#define HAL_IPA_IPA_FRAME_META_H

#include <stdint.h>


#include "IpaStats.h"
#include "mochacamera/Controls.h"

namespace android {

/* Per-frame control state the IPA needs to decide what to update,
 * taken from the frame's settings in StatsProcessStage.
 *
 * Each field defaults to Camera2's "auto" value, so an IPA can treat a
 * request that says nothing as permission to run. */
struct IpaFrameMeta {
    /* Off means the framework controls exposure / gain directly; the
     * IPA must not push either. */
    mocha::camera::AeMode aeMode;

    /* The IPA must hold the last exposure / gain decision. */
    bool aeLock;

    /* Off means the framework controls colour-correction gains
     * directly; presets clip the Bayesian AWB's CT search. */
    mocha::camera::AwbMode awbMode;

    /* The IPA must hold the last WB gains. */
    bool awbLock;

    /* Signed step count in units of Capability::ae.evStep. The IPA
     * scales its target by 2^(steps x step) before driving the AE
     * controller; the manual path applies the same scaling against the
     * requested shutter time (AutoExposureController::parseManualSettings). */
    int32_t aeExposureCompensation;

    /* The manual colour-correction gains, set only for AWB Off when the
     * request carries them, so auto frames keep the IPA's own gains. */
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
        : aeMode(mocha::camera::AeMode::On),
          aeLock(false),
          awbMode(mocha::camera::AwbMode::Auto),
          awbLock(false),
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
