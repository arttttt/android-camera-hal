#ifndef SENSOR_CONFIG_H
#define SENSOR_CONFIG_H

#include <stdint.h>

#include "DelayedControls.h"

namespace android {

/*
 * Per-sensor configuration. Fields split by source:
 *
 *  - Driver-queried at openDevice via VIDIOC_QUERYCTRL. The per-sensor
 *    builders (imx179() / ov5693()) leave these at 0 since
 *    Camera::populateSensorConfigFromDriver() always runs before any
 *    consumer reads them. If QUERYCTRL fails, the HAL won't start AE
 *    anyway — zero is the honest "not yet known" value.
 *
 *  - Sensor-physical properties the driver doesn't expose through
 *    standard V4L2 CIDs. Hardcoded in the per-sensor builder because
 *    that's where the sensor-specific knowledge lives; getting them
 *    from the driver would require custom CIDs or OTP parsing.
 *
 *  - Silicon / convention properties that can't change at runtime.
 */
struct SensorConfig {
    /* Driver-queried — V4L2_CID_FRAME_LENGTH min/max/def. */
    int32_t frameLenDefault;
    int32_t frameLenMax;

    /* Sensor-physical — not exposed via stock V4L2 today. */
    int32_t lineTimeUs;         /* per-line time (µs) at default mode */
    int32_t maxCoarseDiff;      /* frame_length - max_coarse_time */

    /* Gain convention: Q8 fixed-point throughout (256 = 1.0x) for
     * the sensors we ship now. The builder records the unit so
     * callers can reason about it without assuming Q8; drivers that
     * later join with a different encoding fill this per-sensor. */
    int32_t gainUnit;

    /* Driver-queried — V4L2_CID_GAIN min/def/max (Q `gainUnit`). */
    int32_t gainMin;
    int32_t gainDefault;
    int32_t gainMax;

    /* Driver-queried — V4L2_CID_EXPOSURE min/def/max (µs). */
    int32_t exposureMin;
    int32_t exposureDefault;
    int32_t exposureMax;

    /* Silicon property — number of frames between writing a sensor
     * control and the resulting frame being exposed. Not tunable per
     * integrator. Default 2/2 matches the libcamera convention for
     * rolling-shutter CMOS; verify empirically once a real 3A loop
     * drives it. Consumed by DelayedControls. */
    int32_t controlDelay[DelayedControls::COUNT];

    /* --- Helpers --- */

    /* Max exposure (µs) at the given frame_length */
    int32_t maxExposureUs(int32_t frameLen) const {
        return (frameLen - maxCoarseDiff) * lineTimeUs;
    }

    /* Max exposure (µs) at default frame_length (no FPS drop) */
    int32_t maxExposureUsDefault() const {
        return maxExposureUs(frameLenDefault);
    }

    /* Frame_length needed for a given exposure (µs) */
    int32_t frameLenForExposure(int32_t exposureUs) const {
        int32_t fl = exposureUs / lineTimeUs + maxCoarseDiff;
        if (fl < frameLenDefault) fl = frameLenDefault;
        if (fl > frameLenMax) fl = frameLenMax;
        return fl;
    }

    /* Camera2 spec convention: ISO sensitivity scale is anchored at
     * unity sensor gain → ISO `kIsoAtUnityGain`. The framework rounds
     * SENSOR_SENSITIVITY values relative to this anchor; advertise the
     * same anchor in characteristics so the slider numbers match what
     * the per-frame result reports. */
    static constexpr int32_t kIsoAtUnityGain = 100;

    /* Convert ISO (100-based) to sensor gain units */
    int32_t isoToGain(int32_t iso) const {
        int32_t g = (iso / kIsoAtUnityGain) * gainUnit;
        return (g < 1) ? 1 : g;
    }

    /* Convert sensor gain units to ISO (100-based, anchored at unity). */
    int32_t gainToIso(int32_t gain) const {
        if (gainUnit <= 0) return 0;
        return (gain * kIsoAtUnityGain) / gainUnit;
    }

    /* Split target exposure into actual exposure + extra gain (Q8).
     * Keeps exposure within default frame_length, excess → gain. */
    void splitExposureGain(int32_t targetUs, int32_t *outExposureUs,
                           int32_t *outExtraGainQ8) const {
        int32_t maxUs = maxExposureUsDefault();
        if (targetUs <= maxUs) {
            *outExposureUs = targetUs;
            *outExtraGainQ8 = 256; /* 1.0x */
        } else {
            *outExposureUs = maxUs;
            *outExtraGainQ8 = (int32_t)((int64_t)targetUs * 256 / maxUs);
        }
    }

    /* Pre-built configs for known sensors — carry only the fields
     * QUERYCTRL can't give us. Camera::populateSensorConfigFromDriver
     * fills the rest before any consumer reads mSensorCfg. */
    static SensorConfig imx179() {
        return {
            .frameLenDefault = 0,
            .frameLenMax     = 0,
            .lineTimeUs      = 13,
            .maxCoarseDiff   = 6,
            .gainUnit        = 256,
            .gainMin         = 0,
            .gainDefault     = 0,
            .gainMax         = 0,
            .exposureMin     = 0,
            .exposureDefault = 0,
            .exposureMax     = 0,
            .controlDelay    = { 2, 2 },
        };
    }

    static SensorConfig ov5693() {
        return {
            .frameLenDefault = 0,
            .frameLenMax     = 0,
            .lineTimeUs      = 17,
            .maxCoarseDiff   = 6,
            .gainUnit        = 256,
            .gainMin         = 0,
            .gainDefault     = 0,
            .gainMax         = 0,
            .exposureMin     = 0,
            .exposureDefault = 0,
            .exposureMax     = 0,
            .controlDelay    = { 2, 2 },
        };
    }
};

}; /* namespace android */

#endif // SENSOR_CONFIG_H
