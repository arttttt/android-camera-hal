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

/* Exposure and gain envelopes come from SensorConfig at write-time —
 * no compile-time constants. `maxExposureUsDefault()` respects the
 * sensor's default frame_length (so AE doesn't drop FPS by stretching
 * exposure past one frame period), and `gainMax` is the live value
 * V4L2 QUERYCTRL advertised on this driver. Using a HAL-side
 * `maxExposureUs = 200000 µs` constant inflated the AE state ceiling
 * to ~6× what the sensor can physically reach at 30 fps, so a dark
 * scene saturated `lastTotalUs` into phantom territory and AE
 * needed dozens of frames to unwind once the scene brightened —
 * visible as daylight over-exposure + brightness pumping. */

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


/* Knob derivations from the sensor's tuning. No silent fallbacks —
 * if a field is absent or zero, the corresponding BasicIpa member
 * ends up at zero, which naturally disables the matching AWB / AE
 * branch (awbDamping == 0 → EMA freezes, aeDamping == 0 → AE
 * freezes, gated on flagged control modes). The contract is that
 * `active.{ae,awb.v4}` is always populated for any shipping tuning;
 * if it isn't, the IPA does nothing and V4L2 runs on the framework /
 * manual path instead of sneaking in compile-time heuristics. */

float awbParam(const SensorTuning *t,
               float (SensorTuning::AwbParams::* field)) {
    return (t && (t->awbParams().*field) > 0.f) ? t->awbParams().*field : 0.f;
}

/* AE setpoint from the MeanAlg target pair. NVIDIA authors these in
 * the post-gamma 0..255 domain (so 110..120 sits around sRGB middle
 * grey 0.45 post-gamma); our histogram comes off the raw Bayer
 * green channel in linear pre-gamma [0, 1] space. Gamma-decode the
 * midpoint with the standard 2.2 exponent so the setpoint lands
 * near 18 % middle grey linear — (115/255)^2.2 ≈ 0.174 for IMX179,
 * (130/255)^2.2 ≈ 0.227 for OV5693 (different MeanAlg targets per
 * sensor). */
float deriveAeSetpoint(const SensorTuning *t) {
    if (!t || !t->aeParams().loaded) return 0.f;
    const float mid = (t->aeParams().higherTarget
                     + t->aeParams().lowerTarget) * 0.5f;
    if (mid <= 0.f) return 0.f;
    return powf(mid / 255.0f, 2.2f);
}

float deriveAeDamping(const SensorTuning *t) {
    if (!t || !t->aeParams().loaded) return 0.f;
    return t->aeParams().convergeSpeed;
}

float deriveAeRatioMax(const SensorTuning *t) {
    if (!t || !t->aeParams().loaded
     || t->aeParams().maxFstopDeltaPos <= 0.f) return 0.f;
    return powf(2.0f, t->aeParams().maxFstopDeltaPos);
}

float deriveAeRatioMin(const SensorTuning *t) {
    if (!t || !t->aeParams().loaded
     || t->aeParams().maxFstopDeltaNeg <= 0.f) return 0.f;
    return 1.0f / powf(2.0f, t->aeParams().maxFstopDeltaNeg);
}

float deriveAeTolIn(const SensorTuning *t) {
    if (!t || !t->aeParams().loaded) return 0.f;
    return t->aeParams().toleranceIn;
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
      awbMinChannel     (awbParam(sensorTuning,
                                  &SensorTuning::AwbParams::cStatsMinThreshold)),
      awbSceneLightFloor(awbParam(sensorTuning,
                                  &SensorTuning::AwbParams::cStatsDarkThreshold)),
      awbDamping        (awbParam(sensorTuning,
                                  &SensorTuning::AwbParams::smoothingWpTrackingFraction)),
      aeSetpoint (deriveAeSetpoint (sensorTuning)),
      aeDamping  (deriveAeDamping  (sensorTuning)),
      aeRatioMin (deriveAeRatioMin (sensorTuning)),
      aeRatioMax (deriveAeRatioMax (sensorTuning)),
      aeToleranceInStops (deriveAeTolIn (sensorTuning)),
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
      smoothedLuma(0.f),
      smoothedAeMult(1.0f),
      frameCount(0),
      mAeLocked(false),
      mAeConvergedFrames(0) {
    const bool aeOK  = tuning && tuning->aeParams().loaded
                      && aeSetpoint > 0.f && aeDamping > 0.f
                      && aeRatioMin > 0.f && aeRatioMax > 0.f;
    const bool awbOK = tuning && tuning->awbParams().loaded
                      && awbMinChannel > 0.f
                      && awbSceneLightFloor > 0.f
                      && awbDamping > 0.f;
    ALOGD("3A knobs: aeOK=%d aeSetpoint=%.3f aeDamping=%.3f aeRatio=[%.3f,%.3f] "
          "aeTolIn=%.3fst "
          "awbOK=%d awbMinChannel=%.4f awbSceneLightFloor=%.4f awbDamping=%.3f "
          "wbPrior=(%.3f,%.3f) "
          "gainMax=%d gainUnit=%d maxExpDef=%d tuningLoaded=%d",
          aeOK ? 1 : 0,
          (double)aeSetpoint, (double)aeDamping,
          (double)aeRatioMin, (double)aeRatioMax,
          (double)aeToleranceInStops,
          awbOK ? 1 : 0,
          (double)awbMinChannel, (double)awbSceneLightFloor,
          (double)awbDamping, (double)wbRPrior, (double)wbBPrior,
          sensorCfg.gainMax, sensorCfg.gainUnit,
          sensorCfg.maxExposureUsDefault(),
          tuning ? (tuning->isLoaded() ? 1 : 0) : -1);

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
    smoothedLuma   = 0.f;
    smoothedAeMult = 1.0f;
    mAeLocked          = false;
    mAeConvergedFrames = 0;

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
    int diagNValid = -1;  /* set inside the AWB block when it runs */
    if (awbRun) {
        /* Gray-world over rgbMean patches, with saturated / near-black
         * patch exclusion. The pre-WB / pre-CCM domain means a clipped
         * patch skews disproportionately — filtering at the patch
         * level (not per-pixel, which we can't do here) is the usual
         * raw-domain AWB robustness step. */
        float sumR = 0.f, sumG = 0.f, sumB = 0.f;
        int nValid = 0;
        diagNValid = 0;  /* will be set to real nValid below */
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
        diagNValid = nValid;

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

            /* Always damp. The old "first-tick snap" skipped damping
             * on the very first valid tick and let lastWb jump from
             * the FusionLights prior straight to the current frame's
             * gray-world output — a visible WB shock even if the new
             * state was closer to correct. With the priors coming
             * from a real calibration anchor, the EMA at
             * SmoothingWpTrackingFraction = 0.1 crawls to the scene
             * colour in ~20 frames with no single-frame pops. */
            lastWbR = awbDamping * rGain + (1.0f - awbDamping) * lastWbR;
            lastWbB = awbDamping * bGain + (1.0f - awbDamping) * lastWbB;
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
            /* Apply the tuning's gray-line soft-clamp to U before
             * feeding CCT — noise on dark scenes pushes raw
             * ln(G/B) outside the calibrated range, and the clamp
             * snaps it back onto the locus the sensor was
             * characterised over. */
            const float U       = tuning->clampU(logf(lastWbB));
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
        const int32_t diagGainClamped =
            diagGain > sensorCfg.gainMax ? sensorCfg.gainMax
                                          : (diagGain < 1 ? 1 : diagGain);
        ALOGD("3A: frame=%u luma=%.3f nValid=%d awbRun=%d "
              "lastWb=(%.3f,%.3f) wbPrior=(%.3f,%.3f) "
              "Q8=(%u,%u) estCct=%d totalUs=%.0f exp=%d gain=%d gainClamp=%d",
              frameCount, (double)sceneLuma, diagNValid, awbRun ? 1 : 0,
              (double)lastWbR, (double)lastWbB,
              (double)wbRPrior, (double)wbBPrior,
              toQ8(lastWbR), toQ8(lastWbB),
              diagEstCct, (double)lastTotalUs, diagExp,
              diagGain, diagGainClamped);
    }

    /* Manual AE hands exposure / gain authority to the framework.
     * ApplySettingsStage writes its values directly and pushes them
     * into DelayedControls itself for result metadata; an IPA push
     * on the same slot would clobber that. Skip the AE math so last*
     * is frozen at its last auto decision — convergence resumes from
     * there on switch-back.
     *
     * `mAeLocked` (toggled via setAeLock by AF before / after a
     * sweep) freezes the controller the same way: no batch entries
     * are populated, internal state stays put, DelayedControls
     * keeps re-publishing the last queued exposure / gain, and the
     * sensor sits at the converged operating point. Releasing the
     * lock resumes from where convergence left off, not from a
     * stale-mid-update value. */
    if (meta.aeMode == ANDROID_CONTROL_AE_MODE_OFF
     || meta.aeLock == ANDROID_CONTROL_AE_LOCK_ON
     || mAeLocked) {
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

    /* EMA the measured luma before the controller sees it. Static
     * scene + hand-shake + sensor noise would otherwise wiggle raw
     * meanLuma ±30 % frame to frame, which is more than the
     * ToleranceIn dead-band is wide — the controller would enter
     * and exit the dead-band on every other frame and AE would look
     * visibly unstable. Seed on first hit so the filter doesn't
     * spend its start-up ramp dragging from zero. */
    if (smoothedLuma <= 0.f) smoothedLuma = meanLuma;
    else                     smoothedLuma = aeDamping * meanLuma
                                           + (1.0f - aeDamping) * smoothedLuma;
    meanLuma = smoothedLuma;

    /* P-controller toward the setpoint, hard-clamped and EMA-damped.
     * Dead-band the adjustment when luma is already within
     * aeToleranceInStops (from tuning's ae.MeanAlg.ToleranceIn) of
     * the setpoint — stops AE from chasing sensor noise and scene
     * micro-flutter around the target, which would otherwise show
     * up as a visible brightness ripple in "well-exposed" scenes.
     * Tolerance is in f-stops; we compare the log2 of the ratio
     * against it (ratio=1.414 → 0.5 stops, ratio=0.707 → -0.5
     * stops). toleranceIn==0 (tuning missing) disables the gate so
     * BasicIpa behaves as before. */
    float ratio = aeSetpoint / meanLuma;
    if (ratio < aeRatioMin) ratio = aeRatioMin;
    if (ratio > aeRatioMax) ratio = aeRatioMax;

    /* Dead-band from the tuning's ToleranceIn: when luma is already
     * within that many stops of the setpoint, hold the AE state and
     * republish the current split so result metadata stays
     * consistent. No Schmitt latch — the cascaded smoothing below is
     * what keeps AE from jerking on micro-scene flutter; adding an
     * exit-gate on top only made AE visibly freeze once it settled.
     * While held, relax the smoothed multiplier back toward 1.0 so
     * the next tick out of dead-band doesn't resume with a stale
     * ramp. */
    if (aeToleranceInStops > 0.f) {
        const float stops = log2f(ratio);
        const float absStops = stops < 0.f ? -stops : stops;
        if (absStops < aeToleranceInStops) {
            int32_t heldExposureUs, heldExtraGainQ8;
            sensorCfg.splitExposureGain((int32_t)(lastTotalUs + 0.5f),
                                         &heldExposureUs, &heldExtraGainQ8);
            int32_t heldGain = (int32_t)(((int64_t)sensorCfg.gainUnit
                                         * heldExtraGainQ8 + 128) / 256);
            if (heldGain < 1)                 heldGain = 1;
            if (heldGain > sensorCfg.gainMax) heldGain = sensorCfg.gainMax;
            batch.has[DelayedControls::EXPOSURE] = true;
            batch.val[DelayedControls::EXPOSURE] = heldExposureUs;
            batch.has[DelayedControls::GAIN]     = true;
            batch.val[DelayedControls::GAIN]     = heldGain;
            smoothedAeMult = aeDamping * 1.0f
                           + (1.0f - aeDamping) * smoothedAeMult;
            /* Each in-tolerance frame counts toward the convergence
             * report — once the count crosses the threshold,
             * isAeConverged() flips true. */
            if (mAeConvergedFrames < INT32_MAX) mAeConvergedFrames++;
            return batch;
        }
    }
    /* Out of dead-band (or dead-band disabled): AE actively chasing,
     * so we're not stably at setpoint. */
    mAeConvergedFrames = 0;

    /* Cascaded smoothing on the AE per-frame multiplier. The
     * first-order damped ratio is itself EMA'd (same ConvergeSpeed)
     * before being applied to the state. Effect: on a step scene
     * change the per-frame state delta starts at ~damping² of the
     * target (≈1 % at ConvergeSpeed=0.2) and ramps up smoothly,
     * instead of jumping to damping×ratio_clamp (≈8 %) on the very
     * first frame. Convergence envelope stays similar — a full stop
     * is still closed in ~30 frames — but the ride is visibly
     * smooth. */
    const float adjusted = 1.0f + (ratio - 1.0f) * aeDamping;
    smoothedAeMult = aeDamping * adjusted + (1.0f - aeDamping) * smoothedAeMult;
    /* No hard per-frame cap: with the Q8 gain fix on IMX179 (kernel
     * patch) the sensor no longer quantises to whole-stop brackets,
     * so cascade EMA × ratio_clamp × damping already lands under the
     * perceptibility threshold for every individual frame while
     * keeping convergence at ~0.5 s per stop. */

    /* Accumulate in float "µs at unity gain" space. Exposure and
     * gain are only materialised at write-time via
     * SensorConfig::splitExposureGain; storing rounded ints across
     * frames lost ~1/256 of the signal on gainUnit=1 sensors and
     * stalled the EMA at its first rounding step.
     *
     * Clamp to what the sensor can physically deliver at its default
     * frame_length: max exposure × driver-advertised gainMax, both
     * read from SensorConfig. Letting the state exceed this walks AE
     * into a region where every increment is ignored by V4L2 (the
     * driver clamps to gainMax silently), producing lag and a
     * brightness overshoot on scene transitions. */
    const int32_t gainUnit = sensorCfg.gainUnit;
    float newTotal = lastTotalUs * smoothedAeMult;

    const float maxTotal = (float)sensorCfg.maxExposureUsDefault()
                         * (float)sensorCfg.gainMax / (float)gainUnit;
    const float minTotal = (float)sensorCfg.lineTimeUs * 2.0f;
    if (newTotal < minTotal) newTotal = minTotal;
    if (newTotal > maxTotal) newTotal = maxTotal;
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

bool BasicIpa::isAeConverged() const {
    /* Five consecutive in-tolerance frames is enough for the
     * controller's cascade EMA to have a meaningful chance of being
     * settled, while still being short enough that AF doesn't wait
     * a noticeable beat after the first dead-band entry. */
    constexpr int32_t kRequired = 5;
    return mAeConvergedFrames >= kRequired;
}

void BasicIpa::setAeLock(bool lock) {
    mAeLocked = lock;
}

} /* namespace android */
