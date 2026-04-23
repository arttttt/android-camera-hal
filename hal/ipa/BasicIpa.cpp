#include "BasicIpa.h"

#include <math.h>
#include <stdint.h>

#include <system/camera_metadata.h>

#include "IpaFrameMeta.h"
#include "IpaStats.h"
#include "IspPipeline.h"
#include "sensor/SensorConfig.h"
#include "sensor/SensorTuning.h"

#define LOG_TAG "Cam-BasicIpa"
#include <utils/Log.h>

namespace android {

namespace {

/* Exposure envelope used by the AE split. minTotalUs keeps sensor
 * readout honest; maxExposureUs keeps FPS inside the default
 * frame_length. Matches ExposureControl's own clamps so manual /
 * auto agree on reachable values. Sensor-spec, not colour tuning —
 * lives here rather than in the JSON. */
constexpr int64_t minTotalUs     = 100;
constexpr int64_t maxExposureUs  = 200000;

/* Saturation ceiling for per-patch filtering. Patches with any
 * channel above this value are dropped — the missing highlight
 * biases the mean toward the unclipped channel. Algorithm choice,
 * not read from tuning: NVIDIA's CStatsSaturationThreshold
 * represents a different stats-stage metric, and 0.95 in our
 * [0, 1] normalised code is what NeonStatsEncoder is calibrated
 * against. */
constexpr float awbMaxChannel = 0.95f;

/* Minimum valid-patch count required to update the gains. With 256
 * patches a low-contrast scene easily gives > 200 valid; this
 * guard protects the first few frames where the ISP is warming up
 * and pathological all-dark / all-saturated frames where a fresh
 * estimate would just pump the gains. Below the threshold we hold
 * the previous EMA-damped state. */
constexpr int awbMinValidPatches = 32;

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

/* Fallbacks for the tuning-driven knobs (stored as BasicIpa members,
 * resolved at construction). Used when the tuning predates the
 * active.{awb.v4,ae} promotions or ships a zero where the HAL needs
 * a positive value. Match NVIDIA's reserved defaults so behaviour
 * is identical with or without the promotion. */
constexpr float awbMinChannelFallback      = 0.02f;
constexpr float awbSceneLightFloorFallback = 0.05f;
constexpr float awbDampingFallback         = 0.15f;
constexpr float aeSetpointFallback         = 0.45f;   /* matches 115/255 midpoint */
constexpr float aeDampingFallback          = 0.2f;
constexpr float aeRatioMinFallback         = 0.707f;  /* 2^-0.5 fstop */
constexpr float aeRatioMaxFallback         = 1.414f;  /* 2^+0.5 fstop */

float pickAwbParam(const SensorTuning *t,
                   float (SensorTuning::AwbParams::* field),
                   float fallback) {
    if (t && (t->awbParams().*field) > 0.f)
        return t->awbParams().*field;
    return fallback;
}

/* AE setpoint from the MeanAlg target pair. HigherTarget is the
 * target at bright scenes, LowerTarget at dim scenes — scalar
 * consumers (like this AE) take the midpoint as a compromise
 * setpoint. Divides by 255 to match our histogram's [0, 1] scale. */
float deriveAeSetpoint(const SensorTuning *t, float fallback) {
    if (!t || !t->aeParams().loaded) return fallback;
    const float mid = (t->aeParams().higherTarget
                     + t->aeParams().lowerTarget) * 0.5f;
    if (mid <= 0.f) return fallback;
    return mid / 255.0f;
}

float deriveAeDamping(const SensorTuning *t, float fallback) {
    if (t && t->aeParams().loaded && t->aeParams().convergeSpeed > 0.f)
        return t->aeParams().convergeSpeed;
    return fallback;
}

float deriveAeRatioMax(const SensorTuning *t, float fallback) {
    if (t && t->aeParams().loaded && t->aeParams().maxFstopDeltaPos > 0.f)
        return powf(2.0f, t->aeParams().maxFstopDeltaPos);
    return fallback;
}

float deriveAeRatioMin(const SensorTuning *t, float fallback) {
    if (t && t->aeParams().loaded && t->aeParams().maxFstopDeltaNeg > 0.f)
        return 1.0f / powf(2.0f, t->aeParams().maxFstopDeltaNeg);
    return fallback;
}

unsigned toQ8(float x) {
    return (unsigned)(x * 256.0f + 0.5f);
}

/* Mean normalised luma from the green-channel histogram in
 * IpaStats. Matches the metric AE uses so the light gate is
 * calibrated against the same number AE optimises. */
float meanLumaFromHist(const IpaStats &stats) {
    uint32_t count = 0u;
    uint64_t weightedSum = 0u;
    for (int i = 0; i < IpaStats::HIST_BINS; ++i) {
        count       += stats.lumaHist[i];
        weightedSum += (uint64_t)i * stats.lumaHist[i];
    }
    if (count == 0u) return 0.f;
    return (float)weightedSum
         / ((float)count * (float)(IpaStats::HIST_BINS - 1));
}

} /* namespace */

BasicIpa::BasicIpa(const SensorConfig &cfg, IspPipeline *ispPipeline,
                   const SensorTuning *sensorTuning,
                   const float wbGainPrior[3],
                   int16_t *ccmBufQ10)
    : sensorCfg(cfg),
      isp(ispPipeline),
      tuning(sensorTuning),
      ccmBufferQ10(ccmBufQ10),
      awbMinChannel(pickAwbParam(sensorTuning,
                                 &SensorTuning::AwbParams::cStatsMinThreshold,
                                 awbMinChannelFallback)),
      awbSceneLightFloor(pickAwbParam(sensorTuning,
                                       &SensorTuning::AwbParams::cStatsDarkThreshold,
                                       awbSceneLightFloorFallback)),
      awbDamping(pickAwbParam(sensorTuning,
                              &SensorTuning::AwbParams::smoothingWpTrackingFraction,
                              awbDampingFallback)),
      aeSetpoint (deriveAeSetpoint (sensorTuning, aeSetpointFallback)),
      aeDamping  (deriveAeDamping  (sensorTuning, aeDampingFallback)),
      aeRatioMin (deriveAeRatioMin (sensorTuning, aeRatioMinFallback)),
      aeRatioMax (deriveAeRatioMax (sensorTuning, aeRatioMaxFallback)),
      lastTotalUs((float)cfg.exposureDefault * (float)cfg.gainDefault
                  / (float)cfg.gainUnit),
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
      lastWbB(wbBPrior),
      awbFirstTick(true),
      frameCount(0) {
    ALOGD("3A knobs: aeSetpoint=%.3f aeDamping=%.3f aeRatio=[%.3f,%.3f] "
          "awbMinChannel=%.4f awbSceneLightFloor=%.4f awbDamping=%.3f "
          "wbPrior=(%.3f,%.3f)",
          (double)aeSetpoint, (double)aeDamping,
          (double)aeRatioMin, (double)aeRatioMax,
          (double)awbMinChannel, (double)awbSceneLightFloor,
          (double)awbDamping, (double)wbRPrior, (double)wbBPrior);

    /* Seed the shader immediately so the very first frame — before
     * any stats land — renders through the prior's WB gains instead
     * of the IspPipeline's unity defaults. Without this, a session
     * that boots below the dark-scene gate (front cam in a dim room)
     * runs at unity until the gate releases, and the sensor's G-heavy
     * raw response (IMX179 especially) shows up as a green cast.
     * The prior is the calibrated daylight anchor — worst-case-best
     * default across scenes. */
    if (isp) {
        isp->setWbGains(toQ8(lastWbR), wbGainUnityQ8, toQ8(lastWbB));
    }
}

void BasicIpa::reset() {
    lastTotalUs    = (float)sensorCfg.exposureDefault
                   * (float)sensorCfg.gainDefault
                   / (float)sensorCfg.gainUnit;
    lastWbR        = wbRPrior;
    lastWbB        = wbBPrior;
    awbFirstTick   = true;

    /* Re-seed the shader with the priors so the next session starts
     * from the sensor's calibrated daylight anchor even if the first
     * frame is below the AWB gate. WB goes straight to the ISP, CCM
     * takes the same U→CCT→LERP path the per-frame AWB tick uses so
     * cold-start matches where AWB will later converge. Tunings
     * without awb.v4 skip the CCM reseed and leave whatever Camera
     * loaded at build time. */
    if (isp) {
        isp->setWbGains(toQ8(lastWbR), wbGainUnityQ8, toQ8(lastWbB));
    }
    if (tuning && ccmBufferQ10 && tuning->awbParams().loaded) {
        const float U       = logf(wbBPrior);
        const int   estCctK = tuning->estimateCctFromU(U);
        tuning->ccmForCctLerpQ10(estCctK, ccmBufferQ10);
    }
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
     * next AWB commit.
     *
     * Low-light gate: below awbSceneLightFloor the patch means are
     * noise-dominated; computing gains and CCT from them would pump
     * the CCM between CcmSet brackets and show up as a hue swing.
     * Holding last-known-good is the right behaviour there. */
    const float sceneLuma = meanLumaFromHist(stats);
    const bool awbRun = (meta.awbMode == ANDROID_CONTROL_AWB_MODE_AUTO)
                     && (meta.awbLock == ANDROID_CONTROL_AWB_LOCK_OFF)
                     && (isp != nullptr)
                     && !isp->awbLocked()
                     && (sceneLuma >= awbSceneLightFloor);

    /* Throttled diagnostic. Logs the scene-luma, AWB gate status,
     * live gains + Q8 values that end up in IspPipeline, and the
     * derived CCT. Temporary while we reconcile steady-state colour
     * cast — remove once confirmed good. */
    const bool logTick = ((frameCount++ & 0x1f) == 0u);  /* every 32 frames */
    int diagEstCct = 0;
    if (tuning && tuning->awbParams().loaded)
        diagEstCct = tuning->estimateCctFromU(logf(lastWbB));
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

            /* First valid tick of the session: snap directly to the
             * gray-world estimate instead of damping from the prior.
             * Priors are sensor-neutral but have no scene-CCT
             * information, so on a cold start in a warm indoor
             * scene the EMA would spend ~20 frames crawling from
             * the (daylight-ish) prior to the correct gains —
             * visible as a held colour cast at low light, where
             * damping isn't also hiding the drift. Damping resumes
             * on the second tick. */
            if (awbFirstTick) {
                lastWbR = rGain;
                lastWbB = bGain;
                awbFirstTick = false;
            } else {
                lastWbR = awbDamping * rGain + (1.0f - awbDamping) * lastWbR;
                lastWbB = awbDamping * bGain + (1.0f - awbDamping) * lastWbB;
            }
        }

        /* Publish even when no update happened, so lock → unlock
         * resynchronises the shader to last*. Cheap — three stores
         * into the pipeline's uniform cache, next demosaic picks them
         * up. */
        isp->setWbGains(toQ8(lastWbR), wbGainUnityQ8, toQ8(lastWbB));

        /* CCT-driven CCM. Convert the gray-world G/B ratio (= lastWbB
         * in our R-and-B-relative-to-G normalisation) into the
         * NVIDIA AWB-v4 chromaticity U = ln(G/B), pass through the
         * sensor's calibrated U→CCT fit, then LERP between the two
         * CcmSets whose cctK brackets the estimate. Gated on awbRun
         * so manual AWB / lock / AF sweep freeze the CCM in lockstep
         * with the gains — a scene where WB is frozen but CCM keeps
         * drifting would produce visible hue shifts. Requires the
         * tuning to have the awb.v4.{UtoCCT,CCTtoU,LowU,HighU}
         * section; tunings that predate it just keep the boot-time
         * CCM (set by Camera at buildInfrastructure). */
        if (tuning && ccmBufferQ10 && tuning->awbParams().loaded) {
            const float U       = logf(lastWbB);
            diagEstCct          = tuning->estimateCctFromU(U);
            tuning->ccmForCctLerpQ10(diagEstCct, ccmBufferQ10);
        }
    }

    if (logTick) {
        int32_t diagExp = 0, diagExtraQ8 = 256;
        sensorCfg.splitExposureGain((int32_t)(lastTotalUs + 0.5f),
                                     &diagExp, &diagExtraQ8);
        const int32_t diagGain = (int32_t)(((int64_t)sensorCfg.gainUnit
                                           * diagExtraQ8 + 128) / 256);
        ALOGD("3A: frame=%u luma=%.3f awbRun=%d lastWb=(%.3f,%.3f) Q8=(%u,%u) "
              "estCct=%d totalUs=%.0f exp=%d gain=%d",
              frameCount, (double)sceneLuma, awbRun ? 1 : 0,
              (double)lastWbR, (double)lastWbB,
              toQ8(lastWbR), toQ8(lastWbB),
              diagEstCct, (double)lastTotalUs, diagExp, diagGain);
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
    /* Re-use the AWB per-channel floor as the AE noise floor — same
     * semantic (sensor noise prevents reliable readings below this),
     * so both loops share a single tuning knob. */
    if (meanLuma < awbMinChannel) meanLuma = awbMinChannel;

    /* P-controller toward the setpoint, hard-clamped and EMA-damped. */
    float ratio = aeSetpoint / meanLuma;
    if (ratio < aeRatioMin) ratio = aeRatioMin;
    if (ratio > aeRatioMax) ratio = aeRatioMax;
    const float adjusted = 1.0f + (ratio - 1.0f) * aeDamping;

    /* Accumulate in float "µs at unity gain" space. Exposure and
     * gain are only materialised at write-time via
     * SensorConfig::splitExposureGain; storing rounded ints across
     * frames lost ~1/256 of the signal on gainUnit=1 sensors and
     * stalled the EMA at its first rounding step. */
    const int32_t gainUnit = sensorCfg.gainUnit;
    float newTotal = lastTotalUs * adjusted;

    const float maxTotal = (float)maxExposureUs * (float)sensorCfg.gainMax
                         / (float)gainUnit;
    if (newTotal < (float)minTotalUs) newTotal = (float)minTotalUs;
    if (newTotal > maxTotal)          newTotal = maxTotal;
    lastTotalUs = newTotal;

    int32_t newExposureUs;
    int32_t newExtraGainQ8;
    sensorCfg.splitExposureGain((int32_t)(newTotal + 0.5f),
                                 &newExposureUs, &newExtraGainQ8);

    int32_t newGain = (int32_t)(((int64_t)gainUnit * newExtraGainQ8 + 128) / 256);
    if (newGain < 1)                 newGain = 1;
    if (newGain > sensorCfg.gainMax) newGain = sensorCfg.gainMax;

    batch.has[DelayedControls::EXPOSURE] = true;
    batch.val[DelayedControls::EXPOSURE] = newExposureUs;
    batch.has[DelayedControls::GAIN]     = true;
    batch.val[DelayedControls::GAIN]     = newGain;
    return batch;
}

} /* namespace android */
