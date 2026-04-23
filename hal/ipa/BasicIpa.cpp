#include "BasicIpa.h"

#include <stdint.h>

#include <system/camera_metadata.h>

#include "IpaFrameMeta.h"
#include "IpaStats.h"
#include "IspPipeline.h"
#include "sensor/SensorConfig.h"

namespace android {

namespace {

/* Target mean-luma in the raw-Bayer green-channel histogram. Matches
 * classic 18 % photography middle grey once debayered + gamma-encoded;
 * a bit higher (0.35 here) because we're pre-gamma. Tuneable later
 * from SensorTuning if a sensor wants a different setpoint. */
constexpr float aeSetpoint = 0.35f;

/* Per-frame fraction of the gap toward setpoint to close. 0.3 gives
 * ~5 frames to converge on a 2× light change, slow enough to avoid
 * pumping on scene motion. */
constexpr float aeDamping = 0.3f;

/* Bound single-step ratio so a very bright / very dark frame can't
 * slam the sensor into a saturated or blacked-out state in one go. */
constexpr float aeRatioMin = 0.5f;
constexpr float aeRatioMax = 2.0f;

/* Minimum measured luma fed into the ratio. Without a floor a pitch-
 * black frame would produce an unbounded ratio. */
constexpr float aeMeasuredFloor = 0.02f;

/* Exposure envelope used by the AE split. minTotalUs keeps sensor
 * readout honest; maxExposureUs keeps FPS inside the default
 * frame_length. Matches ExposureControl's own clamps so manual /
 * auto agree on reachable values. */
constexpr int64_t minTotalUs     = 100;
constexpr int64_t maxExposureUs  = 200000;

/* Patches below awbMinChannel or above awbMaxChannel in any one
 * channel are dropped from the gray-world sum. The low floor filters
 * out near-black tiles where sensor noise dominates the signal; the
 * high ceiling filters out saturated / near-saturated tiles where
 * the missing highlight biases the mean toward the unclipped
 * channel and drags WB off. rgbMean entries are already in [0, 1]
 * (raw code values normalised by max_code in NeonStatsEncoder). */
constexpr float awbMinChannel = 0.02f;
constexpr float awbMaxChannel = 0.95f;

/* Minimum valid-patch count required to update the gains. With 256
 * patches and the above thresholds, a low-contrast scene easily
 * gives > 200 valid; this guard protects the very first few frames
 * where the ISP is warming up, and pathological all-dark / all-
 * saturated frames where a fresh estimate would just pump the
 * gains. Below the threshold we hold the previous EMA-damped gains. */
constexpr int awbMinValidPatches = 32;

/* Damping for the gain EMA. Slower than AE (0.3) because WB is less
 * perceptually tolerant of oscillation — pumping between tints is
 * more visible than pumping between brightness levels. Matches the
 * old in-shader-pipeline gray-world loop so colour-tracking
 * behaviour stays familiar after the move into BasicIpa. */
constexpr float awbDamping = 0.15f;

/* Final clamp on the gain multipliers, relative to unity (G = 1.0).
 * Matches the [128, 1024] Q8 range the previous estimator used —
 * wide enough to correct for indoor incandescent / outdoor shade,
 * tight enough that a bad estimate on a monochrome scene can't
 * saturate one channel into nonsense. */
constexpr float awbGainMin = 0.5f;
constexpr float awbGainMax = 4.0f;

/* Unity in Q8 — 256 = 1.0x. Emitted for the green channel
 * unconditionally since R / B are expressed relative to G. */
constexpr unsigned wbGainUnityQ8 = 256;

unsigned toQ8(float x) {
    return (unsigned)(x * 256.0f + 0.5f);
}

} /* namespace */

BasicIpa::BasicIpa(const SensorConfig &cfg, IspPipeline *ispPipeline,
                   const float wbGainPrior[3])
    : sensorCfg(cfg),
      isp(ispPipeline),
      lastExposureUs(cfg.exposureDefault),
      lastGain(cfg.gainDefault),
      /* Normalise the R / B priors against G so the shader-side WB
       * (which keeps G at unity) stays consistent. Guard against a
       * zero G entry with the same floor the per-frame AWB uses — a
       * tuning with G == 0 would mean "no prior", and unity is the
       * safest fallback. */
      wbRPrior(wbGainPrior[1] > awbMinChannel
               ? wbGainPrior[0] / wbGainPrior[1] : 1.0f),
      wbBPrior(wbGainPrior[1] > awbMinChannel
               ? wbGainPrior[2] / wbGainPrior[1] : 1.0f),
      lastWbR(wbRPrior),
      lastWbB(wbBPrior) {}

void BasicIpa::reset() {
    lastExposureUs = sensorCfg.exposureDefault;
    lastGain       = sensorCfg.gainDefault;
    lastWbR        = wbRPrior;
    lastWbB        = wbBPrior;
}

DelayedControls::Batch BasicIpa::processStats(uint32_t /*inputSequence*/,
                                               const IpaStats &stats,
                                               const IpaFrameMeta &meta) {
    DelayedControls::Batch batch;
    for (int i = 0; i < DelayedControls::COUNT; ++i) {
        batch.has[i] = false;
        batch.val[i] = 0;
    }

    /* AWB — runs first since it has nothing to do with DelayedControls
     * (WB gains hit the shader directly with no silicon delay) and is
     * orthogonal to AE mode. Held when the framework asks for manual /
     * preset AWB, when the framework locks AWB, or when AF is sweeping
     * (IspPipeline::awbLocked, toggled by AutoFocusController). Preset
     * AWB modes (INCANDESCENT etc.) fall into the "hold" bucket for
     * now — plugging in SensorTuning's per-CCT wbGain priors is the
     * next AWB commit. */
    const bool awbRun = (meta.awbMode == ANDROID_CONTROL_AWB_MODE_AUTO)
                     && (meta.awbLock == ANDROID_CONTROL_AWB_LOCK_OFF)
                     && (isp != nullptr)
                     && !isp->awbLocked();
    if (awbRun) {
        /* Gray-world over rgbMean patches, with saturated / near-black
         * patch exclusion. The pre-WB / pre-CCM domain means a clipped
         * patch skews disproportionately — filtering at the patch
         * level (not per-pixel, which we can't do here) is the usual
         * raw-domain AWB robustness step. */
        float sumR = 0.f, sumG = 0.f, sumB = 0.f;
        int nValid = 0;
        for (int py = 0; py < IpaStats::PATCH_Y; ++py) {
            for (int px = 0; px < IpaStats::PATCH_X; ++px) {
                const float r = stats.rgbMean[py][px][0];
                const float g = stats.rgbMean[py][px][1];
                const float b = stats.rgbMean[py][px][2];
                const float maxCh = r > g ? (r > b ? r : b) : (g > b ? g : b);
                const float minCh = r < g ? (r < b ? r : b) : (g < b ? g : b);
                if (maxCh > awbMaxChannel) continue;
                if (minCh < awbMinChannel) continue;
                sumR += r; sumG += g; sumB += b;
                ++nValid;
            }
        }

        if (nValid >= awbMinValidPatches) {
            const float meanR = sumR / (float)nValid;
            const float meanG = sumG / (float)nValid;
            const float meanB = sumB / (float)nValid;

            /* Gains boost each channel toward the G mean. Divide-by-
             * zero guarded by the awbMinChannel floor above (every
             * summed channel is at least awbMinChannel). */
            float rGain = meanG / meanR;
            float bGain = meanG / meanB;
            if (rGain < awbGainMin) rGain = awbGainMin;
            if (rGain > awbGainMax) rGain = awbGainMax;
            if (bGain < awbGainMin) bGain = awbGainMin;
            if (bGain > awbGainMax) bGain = awbGainMax;

            lastWbR = awbDamping * rGain + (1.0f - awbDamping) * lastWbR;
            lastWbB = awbDamping * bGain + (1.0f - awbDamping) * lastWbB;
        }

        /* Publish even when no update happened, so lock → unlock
         * resynchronises the shader to last*. Cheap — three stores
         * into the pipeline's uniform cache, next demosaic picks them
         * up. */
        isp->setWbGains(toQ8(lastWbR), wbGainUnityQ8, toQ8(lastWbB));
    }

    /* Manual AE hands exposure / gain authority to the framework.
     * ApplySettingsStage writes its values directly and pushes them
     * into DelayedControls itself for result metadata; an IPA push
     * on the same slot would clobber that. Skip the AE math so last*
     * is frozen at its last auto decision — convergence resumes from
     * there on switch-back. */
    if (meta.aeMode == ANDROID_CONTROL_AE_MODE_OFF
     || meta.aeLock == ANDROID_CONTROL_AE_LOCK_ON) {
        return batch;
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
    if (meanLuma < aeMeasuredFloor) meanLuma = aeMeasuredFloor;

    /* P-controller toward the setpoint, hard-clamped and EMA-damped. */
    float ratio = aeSetpoint / meanLuma;
    if (ratio < aeRatioMin) ratio = aeRatioMin;
    if (ratio > aeRatioMax) ratio = aeRatioMax;
    const float adjusted = 1.0f + (ratio - 1.0f) * aeDamping;

    /* Last decision in µs-at-unity-gain: exposureUs × (gain / gainUnit).
     * The IPA owns both axes — next step multiplies this scalar by the
     * adjustment, then splits back into (exposure, gain) via
     * SensorConfig so exposure stays inside the default frame_length. */
    const int64_t gainUnit  = sensorCfg.gainUnit;
    const int64_t lastTotal = (int64_t)lastExposureUs * lastGain / gainUnit;

    int64_t newTotal = (int64_t)((float)lastTotal * adjusted);

    const int64_t maxTotal = maxExposureUs * (int64_t)sensorCfg.gainMax / gainUnit;
    if (newTotal < minTotalUs) newTotal = minTotalUs;
    if (newTotal > maxTotal)   newTotal = maxTotal;

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
