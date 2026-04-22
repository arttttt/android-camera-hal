#include "BasicIpa.h"

#include <stdint.h>

#include "IpaStats.h"
#include "sensor/SensorConfig.h"

namespace android {

namespace {

/* Target mean-luma in the raw-Bayer green-channel histogram. Matches
 * classic 18 % photography middle grey once debayered + gamma-encoded;
 * a bit higher (0.35 here) because we're pre-gamma. Tuneable later
 * from SensorTuning if a sensor wants a different setpoint. */
constexpr float kAeSetpoint = 0.35f;

/* Per-frame fraction of the gap toward setpoint to close. 0.3 gives
 * ~5 frames to converge on a 2× light change, slow enough to avoid
 * pumping on scene motion. */
constexpr float kAeDamping = 0.3f;

/* Bound single-step ratio so a very bright / very dark frame can't
 * slam the sensor into a saturated or blacked-out state in one go. */
constexpr float kAeRatioMin = 0.5f;
constexpr float kAeRatioMax = 2.0f;

/* Minimum measured luma fed into the ratio. Without a floor a pitch-
 * black frame would produce an unbounded ratio. */
constexpr float kAeMeasuredFloor = 0.02f;

/* Exposure envelope used by the AE split. Longer than kMinExposureUs
 * keeps sensor readout honest; shorter than kMaxExposureUs keeps FPS
 * inside the default frame_length. Matches ExposureControl's own
 * clamps so manual / auto agree on reachable values. */
constexpr int64_t kMinTotalUs = 100;
constexpr int64_t kMaxExposureUs = 200000;

} /* namespace */

BasicIpa::BasicIpa(const SensorConfig &cfg)
    : sensorCfg(cfg),
      lastExposureUs(cfg.exposureDefault),
      lastGain(cfg.gainDefault) {}

void BasicIpa::reset() {
    lastExposureUs = sensorCfg.exposureDefault;
    lastGain       = sensorCfg.gainDefault;
}

DelayedControls::Batch BasicIpa::processStats(uint32_t /*inputSequence*/,
                                               const IpaStats &stats) {
    DelayedControls::Batch batch;
    for (int i = 0; i < DelayedControls::COUNT; ++i) {
        batch.has[i] = false;
        batch.val[i] = 0;
    }

    /* Mean bin index of the green-channel histogram over all bins.
     * Dropping the saturated (127) and black (0) bins was tempting to
     * suppress clipping outliers, but in high-contrast scenes
     * (backlit subject, laptop screen in a dark room) the excluded
     * bins actually carry the signal — skipping them made AE chase
     * an over-exposed result because the remaining low-luma bezel
     * dominated. Including everything is a cleaner "matrix-style"
     * mean; saturation explicitly pulls the metric toward 1.0 and AE
     * backs off. */
    uint32_t count       = 0u;
    uint64_t weightedSum = 0u;
    for (int i = 0; i < IpaStats::HIST_BINS; ++i) {
        count       += stats.lumaHist[i];
        weightedSum += (uint64_t)i * stats.lumaHist[i];
    }
    if (count == 0u) return batch;

    const float meanBin = (float)weightedSum / (float)count;
    float meanLuma      = meanBin / (float)(IpaStats::HIST_BINS - 1);
    if (meanLuma < kAeMeasuredFloor) meanLuma = kAeMeasuredFloor;

    /* P-controller toward the setpoint, hard-clamped and EMA-damped. */
    float ratio = kAeSetpoint / meanLuma;
    if (ratio < kAeRatioMin) ratio = kAeRatioMin;
    if (ratio > kAeRatioMax) ratio = kAeRatioMax;
    const float adjusted = 1.0f + (ratio - 1.0f) * kAeDamping;

    /* Last decision in µs-at-unity-gain: exposureUs × (gain / gainUnit).
     * The IPA owns both axes — next step multiplies this scalar by the
     * adjustment, then splits back into (exposure, gain) via
     * SensorConfig so exposure stays inside the default frame_length. */
    const int64_t gainUnit  = sensorCfg.gainUnit;
    const int64_t lastTotal = (int64_t)lastExposureUs * lastGain / gainUnit;

    int64_t newTotal = (int64_t)((float)lastTotal * adjusted);

    const int64_t maxTotal = kMaxExposureUs * (int64_t)sensorCfg.gainMax / gainUnit;
    if (newTotal < kMinTotalUs) newTotal = kMinTotalUs;
    if (newTotal > maxTotal)    newTotal = maxTotal;

    int32_t newExposureUs;
    int32_t newExtraGainQ8;
    sensorCfg.splitExposureGain((int32_t)newTotal, &newExposureUs, &newExtraGainQ8);

    int32_t newGain = (int32_t)((int64_t)gainUnit * newExtraGainQ8 / 256);
    if (newGain < 1)                 newGain = 1;
    if (newGain > sensorCfg.gainMax) newGain = sensorCfg.gainMax;

    lastExposureUs = newExposureUs;
    lastGain       = newGain;

    batch.has[DelayedControls::EXPOSURE] = true;
    batch.val[DelayedControls::EXPOSURE] = newExposureUs;
    batch.has[DelayedControls::GAIN]     = true;
    batch.val[DelayedControls::GAIN]     = newGain;
    return batch;
}

} /* namespace android */
