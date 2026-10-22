#ifndef SENSOR_CONFIG_H
#define SENSOR_CONFIG_H

#include <stdint.h>

#include "DelayedControls.h"

namespace android {

/*
 * Per-sensor configuration. Fields split by source:
 *
 *  - Driver-queried at Session::open via VIDIOC_QUERYCTRL. The per-sensor
 *    builders (imx179() / ov5693()) leave these at 0 since
 *    Session::Impl::populateSensorConfigFromDriver() always runs before any
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
    /* Driver-queried for the configured mode — V4L2_CID_FRAME_LENGTH:
     * its minimum (= default) is the mode's own frame, the frame AE runs
     * at; its maximum the longest the sensor takes, for long manual
     * exposures. */
    int32_t frameLenDefault;
    int32_t frameLenMax;

    /* Driver-queried for the configured mode — V4L2_CID_LINE_TIME_NS,
     * read from the mode's own register table: the line time at the
     * mode's own line length. */
    int32_t lineTimeNs;

    /* Driver-queried for the configured mode — V4L2_CID_LINE_LENGTH
     * (pixel clocks): its minimum (= default) is the mode's own line,
     * its maximum the longest the sensor takes. A longer line is a
     * longer line time, in proportion; it is how an exposure longer
     * than the line counter holds at the mode's line time still fits.
     * lineLengthMax == 0: the driver has no such control. */
    int32_t lineLengthDefault;
    int32_t lineLengthMax;

    /* Sensor-physical — frame_length - max_coarse_time, the margin the
     * drivers clamp the coarse time to. */
    int32_t maxCoarseDiff;

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

    /* Lens — the module's focal length (mm), from the tuning's module
     * block. Static metadata advertises it and every result reports
     * it; it is a fixed property of the lens, focusing or not. */
    float focalLengthMm;

    /* --- Helpers --- */

    /* Max exposure (µs) at the given frame_length */
    int32_t maxExposureUs(int32_t frameLen) const {
        return (int32_t)((int64_t)(frameLen - maxCoarseDiff) * lineTimeNs / 1000);
    }

    /* The line time (ns) at `lineLength` pixel clocks. */
    int64_t lineTimeNsAt(int32_t lineLength) const {
        if (lineLengthDefault <= 0 || lineLength <= 0) return lineTimeNs;
        return (int64_t)lineTimeNs * lineLength / lineLengthDefault;
    }

    /* The period of a frame of `frameLen` lines of `lineLength` (ns). */
    int64_t frameDurationNs(int32_t frameLen, int32_t lineLength) const {
        return (int64_t)frameLen * lineTimeNsAt(lineLength);
    }

    /* Max exposure (µs) at default frame_length (no FPS drop) */
    int32_t maxExposureUsDefault() const {
        return maxExposureUs(frameLenDefault);
    }

    /* The frame a sensor runs one exposure in. */
    struct FrameTiming {
        int32_t frameLen;
        int32_t lineLength;
    };

    /* The frame for an exposure (µs): the mode's own line while the
     * longest frame holds the exposure in it, else the shortest line
     * that does; the lines the exposure takes in that line, rounded up,
     * plus the margin; never below the mode's frame. */
    FrameTiming timingForExposure(int32_t exposureUs) const {
        FrameTiming t{frameLenDefault, lineLengthDefault};
        if (lineTimeNs <= 0) return t;
        const int64_t exposureNs = (int64_t)exposureUs * 1000;
        const int64_t maxLines = frameLenMax - maxCoarseDiff;
        if (lineLengthMax > lineLengthDefault && maxLines > 0 &&
            maxLines * lineTimeNs < exposureNs) {
            int64_t ll = (exposureNs * lineLengthDefault
                          + maxLines * lineTimeNs - 1) / (maxLines * lineTimeNs);
            /* lineTimeNsAt floors: step up until the frame holds it. */
            while (ll < lineLengthMax && maxLines * lineTimeNsAt((int32_t)ll) < exposureNs)
                ++ll;
            t.lineLength = (int32_t)(ll > lineLengthMax ? lineLengthMax : ll);
        }
        const int64_t lineNs = lineTimeNsAt(t.lineLength);
        int64_t fl = (exposureNs + lineNs - 1) / lineNs + maxCoarseDiff;
        if (fl < frameLenDefault) fl = frameLenDefault;
        if (fl > frameLenMax) fl = frameLenMax;
        t.frameLen = (int32_t)fl;
        return t;
    }

    /* Camera2 spec convention: ISO sensitivity scale is anchored at
     * unity sensor gain → ISO `kIsoAtUnityGain`. The framework rounds
     * SENSOR_SENSITIVITY values relative to this anchor; advertise the
     * same anchor in characteristics so the slider numbers match what
     * the per-frame result reports. */
    static constexpr int32_t kIsoAtUnityGain = 100;

    /* Convert ISO (100-based) to sensor gain units, to the nearest
     * unit: the gain is fractional (Q `gainUnit`), so ISO 150 is 1.5x. */
    int32_t isoToGain(int32_t iso) const {
        const int32_t g = (int32_t)(((int64_t)iso * gainUnit
                                     + kIsoAtUnityGain / 2) / kIsoAtUnityGain);
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
     * QUERYCTRL can't give us. Session's populateSensorConfigFromDriver
     * fills the rest before any consumer reads the config. */
    static SensorConfig imx179() {
        return {
            .frameLenDefault = 0,
            .frameLenMax     = 0,
            .lineTimeNs      = 0,
            .lineLengthDefault = 0,
            .lineLengthMax   = 0,
            .maxCoarseDiff   = 6,
            .gainUnit        = 256,
            .gainMin         = 0,
            .gainDefault     = 0,
            .gainMax         = 0,
            .exposureMin     = 0,
            .exposureDefault = 0,
            .exposureMax     = 0,
            /* Measured on mocha (LOS 19.1, 20.10.2026): a value written
             * on request N shows in frame N+3, 13 steps of 14 in a
             * square-wave trace. The libcamera-style 2 this replaced
             * was never measured here, and our writes are not tied to
             * start-of-frame the way libcamera's are, so its number
             * does not transfer. */
            .controlDelay    = { 3, 3, 3, 3 },
            .focalLengthMm   = 0.f,
        };
    }

    static SensorConfig ov5693() {
        return {
            .frameLenDefault = 0,
            .frameLenMax     = 0,
            .lineTimeNs      = 0,
            .lineLengthDefault = 0,
            .lineLengthMax   = 0,
            .maxCoarseDiff   = 6,
            .gainUnit        = 256,
            .gainMin         = 0,
            .gainDefault     = 0,
            .gainMax         = 0,
            .exposureMin     = 0,
            .exposureDefault = 0,
            .exposureMax     = 0,
            /* Measured on mocha (20.10.2026) with the same square-wave
             * trace as IMX179: request N shows in frame N+3 on all five
             * transitions. The scene was dim, so the luma ratio was off,
             * but the frame of the step was not. */
            .controlDelay    = { 3, 3, 3, 3 },
            .focalLengthMm   = 0.f,
        };
    }
};

}; /* namespace android */

#endif // SENSOR_CONFIG_H
