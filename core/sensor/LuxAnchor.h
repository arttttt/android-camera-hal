#ifndef ISP_SENSOR_LUX_ANCHOR_H
#define ISP_SENSOR_LUX_ANCHOR_H

namespace android {

/* Single-point calibration anchor for the AE-driven lux index.
 * At a known scene illumination of `lux` lumens-per-area, the
 * sensor's AE converges to total exposure (exposure × gain at
 * unity-gain units) `totalUs` with mean pre-gamma luma `sceneLuma`
 * in [0, 1]. The runtime per-frame lux is back-computed from the
 * current AE state against this triple — see
 * AutoExposureController::computeLuxIndex.
 *
 * Lives in a separate header (rather than nested in SensorTuning's
 * AeParams) so the AE controller can take it as a value type
 * without including the full SensorTuning surface. */
struct LuxAnchor {
    float lux;
    float totalUs;
    float sceneLuma;
};

} /* namespace android */

#endif /* ISP_SENSOR_LUX_ANCHOR_H */
