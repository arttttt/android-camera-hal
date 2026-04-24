#ifndef SENSOR_CONFIG_H
#define SENSOR_CONFIG_H

#include <stdint.h>

#include "DelayedControls.h"

namespace android {

/*
 * Per-sensor configuration — eliminates hardcoded constants.
 * Populated at configure time from driver queries + known sensor data.
 */
struct SensorConfig {
    /* Timing (from driver queryControl or mode tables) */
    int32_t frameLenDefault;    /* default frame_length (lines) */
    int32_t frameLenMax;        /* max allowed frame_length (capped for VI) */
    int32_t lineTimeUs;         /* approx. line time in µs (pix_clk / line_length) */
    int32_t maxCoarseDiff;      /* frame_length - max_coarse_time */

    /* Gain (sensor-specific encoding) */
    int32_t gainUnit;           /* gain value for 1.0x: 1 for IMX179, 256 for OV5693 */
    int32_t gainDefault;        /* initial gain: 8 for IMX179, 2048 for OV5693 */
    int32_t gainMax;            /* max gain from driver */

    /* Defaults */
    int32_t exposureDefault;    /* initial exposure in µs */

    /* Number of frames between writing a sensor control and the
     * resulting frame being exposed. Silicon property — not tunable
     * per integrator. Default 2/2 follows the libcamera convention
     * for rolling-shutter CMOS; verify empirically once a real 3A
     * loop drives it. Consumed by DelayedControls. */
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

    /* Convert ISO (100-based) to sensor gain units */
    int32_t isoToGain(int32_t iso) const {
        int32_t g = (iso / 100) * gainUnit;
        return (g < 1) ? 1 : g;
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

    /* Pre-built configs for known sensors */
    static SensorConfig imx179() {
        return {
            .frameLenDefault = 2510,
            .frameLenMax     = 7500,
            .lineTimeUs      = 13,
            .maxCoarseDiff   = 6,
            /* Q8 (256 = 1.00x) to match the kernel driver post-patch.
             * gainMax tracks the kernel's IMX179_MAX_GAIN = 16384
             * (64x); gainDefault is 256 (1x) so AE starts at unity
             * instead of 8x — QUERYCTRL on configureStreams still
             * overrides gainMax from the driver if we ever run
             * against an older kernel. */
            .gainUnit        = 256,
            .gainDefault     = 256,
            .gainMax         = 64 * 256,
            .exposureDefault = 30000,
            .controlDelay    = { 2, 2 },
        };
    }

    static SensorConfig ov5693() {
        return {
            .frameLenDefault = 2510,
            .frameLenMax     = 7500,
            .lineTimeUs      = 17,
            .maxCoarseDiff   = 6,
            .gainUnit        = 256,
            .gainDefault     = 2048,
            .gainMax         = 65535,
            .exposureDefault = 30000,
            .controlDelay    = { 2, 2 },
        };
    }
};

}; /* namespace android */

#endif // SENSOR_CONFIG_H
