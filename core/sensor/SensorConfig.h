#ifndef SENSOR_CONFIG_H
#define SENSOR_CONFIG_H

#include <math.h>
#include <stdint.h>

#include <optional>

#include "DelayedControls.h"
#include "mochacamera/Controls.h"

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

    /* Silicon property, in sensor frames, per control -- libcamera's
     * SensorDelays: a control written inside frame k (the capture
     * engine writes as the frame's start is reported) is in effect on
     * frame k + controlDelay. Not tunable per integrator. Consumed by
     * DelayedControls. */
    int32_t controlDelay[DelayedControls::COUNT];

    /* Silicon property: the frame after one in which the line length
     * is written new runs without a frame start -- the receiver sees
     * none, and the frame written in runs on through it. The delays
     * still count it. */
    bool lineLengthHidesFrame = false;

    /* The most digital gain the ISP adds past the analog maximum, Q8
     * (kDigitalGainUnit): x4, ISO 6400 over the sensors' 1600 -- the
     * RPi AGC's default max_digital_gain. */
    int32_t digitalGainMax = 4 * 256;

    /* Lens — the module's focal length (mm), from the tuning's module
     * block. Static metadata advertises it and every result reports
     * it; it is a fixed property of the lens, focusing or not. */
    float focalLengthMm;

    /* The pixel pitch (um), a property of the sensor model: the pixel
     * array times it is the array's physical size. */
    float pixelSizeUm;

    /* --- Helpers --- */

    /* How long a frame on `b` (every control set) runs: its frame
     * length, or its exposure and the coarse margin where that is
     * longer -- the sensor stretches a frame to the exposure it holds. */
    int64_t frameSpanNs(const DelayedControls::Batch &b) const {
        const int64_t lineNs = lineTimeNsAt(b.val[DelayedControls::LINE_LENGTH]);
        const int64_t frame  = (int64_t)b.val[DelayedControls::FRAME_LENGTH] * lineNs;
        const int64_t exposure = (int64_t)b.val[DelayedControls::EXPOSURE] * 1000
                               + (int64_t)maxCoarseDiff * lineNs;
        return frame > exposure ? frame : exposure;
    }

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

    /* When a frame's first line began its exposure, ns, from when its
     * last line reached memory: before that, the read-out of `lines`
     * lines and the exposure, at the frame's own controls (`inEffect`,
     * every control set). */
    int64_t exposureStartNs(int64_t frameEndNs,
                            const DelayedControls::Batch &inEffect,
                            uint32_t lines) const {
        const int64_t readout = (int64_t)lines
            * lineTimeNsAt(inEffect.val[DelayedControls::LINE_LENGTH]);
        const int64_t exposure =
            (int64_t)inEffect.val[DelayedControls::EXPOSURE] * 1000;
        return frameEndNs - readout - exposure;
    }

    /* What configure writes before STREAMON, every control set: what a
     * frame runs on before the first write lands. */
    DelayedControls::Batch configureDefaults() const {
        DelayedControls::Batch b;
        b.val[DelayedControls::EXPOSURE]     = exposureDefault;
        b.val[DelayedControls::GAIN]         = gainDefault;
        b.val[DelayedControls::FRAME_LENGTH] = frameLenDefault;
        b.val[DelayedControls::LINE_LENGTH]  = lineLengthDefault;
        b.val[DelayedControls::DIGITAL_GAIN] = kDigitalGainUnit;
        for (bool &h : b.has) h = true;
        return b;
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

    /* The frame for an exposure (µs), at the mode's own line: the lines
     * the exposure takes, rounded up, plus the margin; never below the
     * mode's frame, nor past the longest. The line is not stretched for
     * a longer one: an exposure past the longest frame is cut to it
     * (maxExposureUs) -- on mocha a line length write costs a hidden
     * frame and the frames around it (lineLengthHidesFrame), seconds
     * each, for exposures these sensors gain nothing from. */
    FrameTiming timingForExposure(int32_t exposureUs) const {
        FrameTiming t{frameLenDefault, lineLengthDefault};
        if (lineTimeNs <= 0) return t;
        const int64_t exposureNs = (int64_t)exposureUs * 1000;
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

    /* The ISP's digital gain (DelayedControls::DIGITAL_GAIN), Q8. */
    static constexpr int32_t kDigitalGainUnit = 256;

    /* ISO for an analog `gain` and the ISP's `digitalGain` on top: what
     * SENSOR_SENSITIVITY reports, the whole gain before processing. */
    int32_t sensitivityIso(int32_t gain, int32_t digitalGain) const {
        return (int32_t)((int64_t)gainToIso(gain) * digitalGain / kDigitalGainUnit);
    }

    /* The frame lengths (lines, at the mode's line) auto exposure may
     * run between. */
    struct FrameLimits {
        int32_t minLen;
        int32_t maxLen;
    };

    /* The mode's own frame, and nothing else: the fps kept. */
    FrameLimits modeFrame() const {
        return FrameLimits{frameLenDefault, frameLenDefault};
    }

    /* For Camera2's AE target fps range: no shorter than 1 / max nor
     * the mode's frame, no longer than 1 / min nor the longest the
     * sensor runs. Without a range, the mode's frame. A max the mode's
     * own rate rounds to (30 for its 30.02) is the mode's frame. */
    FrameLimits frameLimits(const std::optional<mocha::camera::FpsRange> &range) const {
        if (!range || range->min <= 0 || range->max <= 0 || lineTimeNs <= 0)
            return modeFrame();
        const int64_t lineNs = lineTimeNsAt(lineLengthDefault);
        int64_t minLen = 1000000000LL / range->max / lineNs;
        int64_t maxLen = 1000000000LL / range->min / lineNs;
        if (minLen * 100 <= (int64_t)frameLenDefault * 101) minLen = frameLenDefault;
        if (frameLenMax > 0 && minLen > frameLenMax) minLen = frameLenMax;
        if (frameLenMax > 0 && maxLen > frameLenMax) maxLen = frameLenMax;
        if (maxLen < minLen) maxLen = minLen;
        return FrameLimits{(int32_t)minLen, (int32_t)maxLen};
    }

    /* The longest exposure (µs) within `limits`. */
    int32_t maxExposureUs(const FrameLimits &limits) const {
        const int32_t us = maxExposureUs(limits.maxLen);
        return exposureMax > 0 && us > exposureMax ? exposureMax : us;
    }

    /* The most exposure (µs at unity gain) auto exposure reaches within
     * `limits`: the longest exposure at every gain. */
    float maxTotalUs(const FrameLimits &limits) const {
        return (float)maxExposureUs(limits)
             * (float)gainMax / (float)gainUnit
             * (float)digitalGainMax / (float)kDigitalGainUnit;
    }

    /* One exposure as the sensor and the ISP make it, and the frame it
     * runs in. */
    struct ExposureSplit {
        int32_t exposureUs;
        int32_t gain;
        int32_t digitalGain;
        int32_t frameLen;
    };

    /* An exposure total (µs at unity gain) divided as libcamera's AGC
     * divides it: the exposure first, its frame lengthened for it up to
     * `limits` (the fps given up only as far as the range allows), then
     * the analog gain up to its maximum, then the ISP's digital gain for
     * the rest, up to digitalGainMax. The frame is the exposure's,
     * never shorter than `limits` allow. */
    ExposureSplit splitExposure(float totalUs, const FrameLimits &limits) const {
        ExposureSplit s;
        const int32_t maxUs = maxExposureUs(limits);
        s.exposureUs = totalUs < (float)maxUs ? (int32_t)(totalUs + 0.5f) : maxUs;
        if (s.exposureUs < 1) s.exposureUs = 1;
        s.frameLen = frameLenFor(s.exposureUs, limits);
        const float rest = totalUs / (float)s.exposureUs;
        int32_t g = (int32_t)lroundf(rest * (float)gainUnit);
        const int32_t lo = gainMin > 0 ? gainMin : 1;
        if (g < lo)      g = lo;
        if (g > gainMax) g = gainMax;
        s.gain = g;
        int32_t d = (int32_t)lroundf(rest * (float)gainUnit / (float)g
                                     * (float)kDigitalGainUnit);
        if (d < kDigitalGainUnit) d = kDigitalGainUnit;
        if (d > digitalGainMax)   d = digitalGainMax;
        s.digitalGain = d;
        return s;
    }

    /* In the mode's own frame. */
    ExposureSplit splitExposure(float totalUs) const {
        return splitExposure(totalUs, modeFrame());
    }

    /* The frame an exposure (µs) takes at the mode's line -- its lines,
     * rounded up, and the margin -- kept within `limits`. */
    int32_t frameLenFor(int32_t exposureUs, const FrameLimits &limits) const {
        if (lineTimeNs <= 0) return limits.minLen;
        const int64_t lineNs = lineTimeNsAt(lineLengthDefault);
        int64_t fl = ((int64_t)exposureUs * 1000 + lineNs - 1) / lineNs + maxCoarseDiff;
        if (fl < limits.minLen) fl = limits.minLen;
        if (fl > limits.maxLen) fl = limits.maxLen;
        return (int32_t)fl;
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
            /* Measured on mocha (22.10.2026, FRAME_START traces, 0.5 s
             * manual stills): frame and line length written 12.8 ms
             * into frame W stretched W + 1, not W; the exposure reached
             * the image of W + 2 -- the rows of W + 2 integrate while
             * W + 1 runs. OV5693 the same, with and without a line
             * length change. Gain is not measured apart from the
             * exposure (both written together); it is taken with it,
             * as the sensors hold it to the exposure's frame
             * (onsemi AR1335: "the gain update is held off by one
             * frame"). */
            .controlDelay    = { 2, 2, 1, 1, 2 },
            /* Measured on mocha (23.10.2026, FRAME_START traces): the
             * line length alone written 3440 -> 5000 and back, 19 to
             * 205 ms into a frame, made that frame's start-to-start
             * its own period and the new one's (200 + 291 ms, 291 +
             * 200 ms), at any phase. A 2 s still, written with the
             * frame length, ran 200 + 2000 ms, then 2000 + 870 ms:
             * the delays counted over the hidden frames, the second
             * stretched to the exposure still held. */
            .lineLengthHidesFrame = true,
            .focalLengthMm   = 0.f,
            /* Sony's IMX179 figures: 1.4 um pixels. */
            .pixelSizeUm     = 1.4f,
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
            /* Measured on mocha with IMX179 (22.10.2026): frame and
             * line length written 14.6 ms into frame W stretched
             * W + 1; the exposure reached the image of W + 2. Gain as
             * for IMX179. */
            .controlDelay    = { 2, 2, 1, 1, 2 },
            .focalLengthMm   = 0.f,
            /* OmniVision's OV5693 figures: 1.4 um pixels. */
            .pixelSizeUm     = 1.4f,
        };
    }
};

}; /* namespace android */

#endif // SENSOR_CONFIG_H
