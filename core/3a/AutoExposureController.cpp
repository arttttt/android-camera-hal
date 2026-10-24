#include "AutoExposureController.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include <algorithm>

#include "ipa/IpaFrameMeta.h"
#include "ipa/IpaStats.h"
#include "sensor/DelayedControls.h"
#include "sensor/SensorConfig.h"
#include "sensor/SensorTuning.h"

namespace android {

namespace {

/* Translate an EV compensation count into a multiplier on exposure:
 * 2^(count x step), the step being Capability::ae.evStep (1/3 stop
 * today). Zero stays exactly 1.0 so the unmodified path does not
 * introduce a powf rounding. The auto path scales its setpoint by it,
 * the manual path the requested shutter time. */
float evCompFactor(int32_t evCompSteps, mocha::camera::Rational step) {
    if (evCompSteps == 0) return 1.0f;
    return powf(2.0f, (float)evCompSteps * step.value());
}

inline int32_t clampInt(int32_t v, int32_t lo, int32_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
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

/* HAL-specific override, not from NVIDIA .isp — see SensorTuning::
 * AeParams::closeSpeedZone. Returned as-is (zero / missing →
 * controller disables the asymmetric boost). */
float deriveAeCloseSpeedZone(const SensorTuning *t) {
    if (!t) return 0.f;
    return t->aeParams().closeSpeedZone;
}


/* Plain mean of the per-patch green channel from IpaStats::rgbMean
 * over the active focus ROI. Same linear pre-WB / pre-CCM domain as
 * the AE setpoint, so the controller compares like with like.
 *
 * Spot-meter shape: patches outside meta.meteringRoi contribute zero
 * weight. Without a tap meta defaults to the project's centre 8×8
 * (IpaStats::FOCUS_ROI_*) so AE meters the centre as a 64-patch
 * average. On a tap the rectangle moves to the user's chosen subject
 * (5×5 minimum from AutoFocusController) and AE meters precisely
 * those patches. */
float meanLumaInRoi(const IpaStats &stats, const IpaFrameMeta &meta) {
    int pyLo = std::max(meta.meteringRoi.pyLo, 0);
    int pyHi = std::min(meta.meteringRoi.pyHi, IpaStats::PATCH_Y);
    int pxLo = std::max(meta.meteringRoi.pxLo, 0);
    int pxHi = std::min(meta.meteringRoi.pxHi, IpaStats::PATCH_X);

    int   count = 0;
    float sum   = 0.f;
    for (int py = pyLo; py < pyHi; ++py) {
        for (int px = pxLo; px < pxHi; ++px) {
            sum += stats.rgbMean[py][px][1];
            ++count;
        }
    }
    if (count <= 0) return 0.f;
    return sum / (float)count;
}

/* Highlight inter-quantile mean — average of the brightest top 2 %
 * patches inside the AE focus ROI, computed as max-of-channels in
 * the post-WB domain. Drives the highlight-protection candidate.
 *
 * Why post-WB max, not raw green: stats live pre-WB, but downstream
 * the demosaic shader applies WB gains. A patch with raw G=0.5 /
 * B=0.55 already clips on the B channel after a 1.6× WB gain.
 * Picking the channel that clips first (max-of-three after WB) is
 * the honest "what will saturate downstream" metric.
 *
 * topK = max(1, count × 2 / 100). Default centre 8×8 ROI (64
 * patches) → topK = 1, IQM degenerates to single-max. Full-frame
 * ROI (256) → topK = 5, matching RPi q=0.98..1.0 spec. */
float top2pcMaxPostWbMean(const IpaStats &stats, const IpaFrameMeta &meta,
                          float wbR, float wbB) {
    int pyLo = std::max(meta.meteringRoi.pyLo, 0);
    int pyHi = std::min(meta.meteringRoi.pyHi, IpaStats::PATCH_Y);
    int pxLo = std::max(meta.meteringRoi.pxLo, 0);
    int pxHi = std::min(meta.meteringRoi.pxHi, IpaStats::PATCH_X);

    const int count = (pyHi - pyLo) * (pxHi - pxLo);
    if (count <= 0) return 0.f;

    constexpr int kMaxTopK = 5;
    int topK = (count * 2) / 100;
    if (topK < 1)        topK = 1;
    if (topK > kMaxTopK) topK = kMaxTopK;

    float top[kMaxTopK];
    for (int i = 0; i < topK; ++i) top[i] = 0.f;
    for (int py = pyLo; py < pyHi; ++py) {
        for (int px = pxLo; px < pxHi; ++px) {
            const float r = stats.rgbMean[py][px][0] * wbR;
            const float g = stats.rgbMean[py][px][1];
            const float b = stats.rgbMean[py][px][2] * wbB;
            const float maxCh = r > g ? (r > b ? r : b)
                                       : (g > b ? g : b);
            if (maxCh <= top[topK - 1]) continue;
            int j = topK - 1;
            while (j > 0 && top[j - 1] < maxCh) {
                top[j] = top[j - 1];
                --j;
            }
            top[j] = maxCh;
        }
    }
    float sum = 0.f;
    for (int i = 0; i < topK; ++i) sum += top[i];
    return sum / (float)topK;
}

constexpr int32_t kAeConvergedFramesRequired = 5;

/* One AE decision: exposure, analog and digital gain, and the frame
 * the split chose for the exposure -- lengthened only as far as the
 * request's fps range allows. It names the line length too: the driver
 * keeps whatever was last written, so a frame a manual long exposure
 * stretched would otherwise stay stretched. */
void fillAeBatch(DelayedControls::Batch *batch, const SensorConfig &cfg,
                 const SensorConfig::ExposureSplit &split) {
    batch->has[DelayedControls::EXPOSURE]     = true;
    batch->val[DelayedControls::EXPOSURE]     = split.exposureUs;
    batch->has[DelayedControls::GAIN]         = true;
    batch->val[DelayedControls::GAIN]         = split.gain;
    batch->has[DelayedControls::DIGITAL_GAIN] = true;
    batch->val[DelayedControls::DIGITAL_GAIN] = split.digitalGain;
    batch->has[DelayedControls::FRAME_LENGTH] = true;
    batch->val[DelayedControls::FRAME_LENGTH] = split.frameLen;
    batch->has[DelayedControls::LINE_LENGTH]  = true;
    batch->val[DelayedControls::LINE_LENGTH]  = cfg.lineLengthDefault;
}

/* A per-frame smoothing coefficient applied once over `frames` frames:
 * leaving (1 - k) of the error per frame leaves (1 - k)^n over n. */
inline float overFrames(float perFrame, int frames) {
    if (frames <= 1) return perFrame;
    return 1.0f - powf(1.0f - perFrame, (float)frames);
}

} /* namespace */

AutoExposureController::AutoExposureController(const SensorConfig &cfg,
                                                const SensorTuning *tuning,
                                                mocha::camera::Rational evStep)
    : sensorCfg(cfg),
      evStep(evStep),
      aeSetpoint(deriveAeSetpoint(tuning)),
      aeDamping(deriveAeDamping(tuning)),
      aeRatioMin(deriveAeRatioMin(tuning)),
      aeRatioMax(deriveAeRatioMax(tuning)),
      aeCloseSpeedZone(deriveAeCloseSpeedZone(tuning)),
      lumaNoiseFloor(
          tuning && tuning->awbParams().cStatsMinThreshold > 0.f
          ? tuning->awbParams().cStatsMinThreshold : 0.f),
      luxAnchor(tuning ? tuning->aeParams().luxAnchor
                       : Optional<LuxAnchor>()),
      filteredTotalUs((float)cfg.exposureDefault * (float)cfg.gainDefault
                      / (float)cfg.gainUnit),
      smoothedLuma(0.f),
      smoothedScene(0.f),
      lastEvComp(0),
      lockedBiasedTotalUs(0.f),
      aeLockHeld(false),
      aeConvergedFrames(0) {
}

void AutoExposureController::reset() {
    filteredTotalUs     = (float)sensorCfg.exposureDefault
                        * (float)sensorCfg.gainDefault
                        / (float)sensorCfg.gainUnit;
    smoothedLuma        = 0.f;
    smoothedScene       = 0.f;
    lastEvComp          = 0;
    lockedBiasedTotalUs = 0.f;
    aeLockHeld          = false;
    aeConvergedFrames   = 0;
}

bool AutoExposureController::isConverged() const {
    return aeConvergedFrames >= kAeConvergedFramesRequired;
}

void AutoExposureController::setLock(bool lock) {
    aeLockHeld = lock;
}

float AutoExposureController::computeLuxIndex(float currentTotalUs,
                                                float currentLuma) const {
    if (!luxAnchor)                    return 0.f;
    if (currentTotalUs       < 1.f)    return 0.f;
    if (luxAnchor->sceneLuma < 1e-6f)  return 0.f;
    /* Per-frame back-compute from the AE-converged state.
     *   sensorY = k × lux × totalUs   (sensor sensitivity k constant)
     * Solving at the anchor for k, then for current lux:
     *   lux = anchor.lux × (anchor.totalUs / currentTotalUs)
     *                    × (currentLuma   / anchor.sceneLuma)
     * In steady state currentLuma ≈ AE setpoint, so the lux signal
     * reduces to the totalUs ratio scaled by the anchor — robust to
     * the AE controller's dead-band noise. */
    return (luxAnchor->lux * currentLuma * luxAnchor->totalUs)
         / (luxAnchor->sceneLuma * currentTotalUs);
}

ExposureWriteValues AutoExposureController::parseManualSettings(
        const mocha::camera::FrameSettings &settings,
        const SensorConfig                 &cfg,
        mocha::camera::Rational             evStep) {
    int32_t exposureUs = cfg.exposureDefault;
    if (settings.sensor.exposureTimeNs)
        exposureUs = (int32_t)(*settings.sensor.exposureTimeNs / 1000);

    /* EV compensation applied on top of the requested exposure. */
    if (settings.ae.evCompensationSteps != 0) {
        exposureUs = (int32_t)lroundf(
                (float)exposureUs
                * evCompFactor(settings.ae.evCompensationSteps, evStep));
    }

    /* Driver-queried envelope; populateSensorConfigFromDriver fills
     * exposureMin / exposureMax from V4L2_CID_EXPOSURE QUERYCTRL. */
    exposureUs = clampInt(exposureUs, cfg.exposureMin, cfg.exposureMax);

    int32_t gain = cfg.gainUnit;  /* 1.0x baseline */
    if (settings.sensor.sensitivityIso)
        gain = cfg.isoToGain(*settings.sensor.sensitivityIso);
    /* Past the analog maximum, the ISP's digital gain makes up the
     * rest: SENSOR_SENSITIVITY is the whole gain before processing. */
    const int32_t analogMax = cfg.gainMax > 0 ? cfg.gainMax : gain;

    ExposureWriteValues out;
    if (settings.ae.mode == mocha::camera::AeMode::Off) {
        /* Manual AE: app told us exactly what to do. Honour the
         * requested exposure verbatim; if it doesn't fit the default
         * frame_length, grow frame_length so it does (FPS drops
         * accordingly — that's the user's choice when they set a 1 s
         * shutter). No splitExposure, no fps preservation. */
        const SensorConfig::FrameTiming t = cfg.timingForExposure(exposureUs);
        /* Past the longest frame, the longest it holds: what the
         * sensor runs, as the record must say. */
        const int32_t longest = cfg.maxExposureUs(cfg.frameLenMax);
        out.exposureUs = longest > 0 && exposureUs > longest ? longest : exposureUs;
        out.frameLen   = t.frameLen;
        out.lineLength = t.lineLength;
        out.gain       = clampInt(gain, cfg.gainMin, analogMax);
        int32_t d = (int32_t)(((int64_t)gain * SensorConfig::kDigitalGainUnit
                               + out.gain / 2) / out.gain);
        out.digitalGain = clampInt(d, SensorConfig::kDigitalGainUnit, cfg.digitalGainMax);
    } else {
        /* Auto AE cold-start fallback (IPA hasn't pushed yet): divided
         * as AE divides it, within the request's fps range. */
        const SensorConfig::ExposureSplit s = cfg.splitExposure(
                (float)exposureUs * (float)gain / (float)cfg.gainUnit,
                cfg.frameLimits(settings.ae.fpsRange),
                cfg.flickerLines(settings.ae.antibanding));
        out.exposureUs  = s.exposureUs;
        out.gain        = s.gain;
        out.digitalGain = s.digitalGain;
        out.frameLen    = s.frameLen;
        out.lineLength  = cfg.lineLengthDefault;
    }
    return out;
}

AeResult AutoExposureController::process(const IpaStats     &stats,
                                          const IpaFrameMeta &meta,
                                          float               currentWbR,
                                          float               currentWbB,
                                          float               appliedTotalUs,
                                          int                 framesElapsed) {
    const float damping = overFrames(aeDamping, framesElapsed);
    /* How long the frame may run: the mode's, or as far as the
     * request's fps range lets it, for a longer exposure before gain. */
    const SensorConfig::FrameLimits limits = sensorCfg.frameLimits(meta.aeFpsRange);
    /* And in whole half periods of the mains, under flickering light. */
    const int32_t flickerLines = sensorCfg.flickerLines(meta.aeAntibanding);
    AeResult out;
    DelayedControls::Batch &batch = out.batch;
    for (int i = 0; i < DelayedControls::COUNT; ++i) {
        batch.has[i] = false;
        batch.val[i] = 0;
    }

    /* Highlight-protection signal. Computed early so it lands in
     * AeResult for the diagnostic log even when the auto path
     * returns via the AE_LOCK branch. */
    const float iqmHighlight = top2pcMaxPostWbMean(stats, meta,
                                                   currentWbR, currentWbB);
    out.iqmHighlight = iqmHighlight;

    /* AE locked — by the framework's AE_LOCK or by AF (across a
     * sweep). Hold the converged operating point: keep
     * `filteredTotalUs` untouched, but **re-publish the held
     * exposure / gain every frame**, so the exposure target stays
     * on the converged values -- the EV rescale below can move them
     * while locked.
     *
     * EV compensation still applies as an additive offset on top of
     * the locked level — `filteredTotalUs` already reflects whatever
     * EV was active at convergence (`lastEvComp`); rescale by the
     * ratio `factor(current) / factor(lastEvComp)` so dragging the
     * EV slider while locked moves exposure proportionally instead
     * of being silently latched. */
    if (meta.aeLock || aeLockHeld) {
        const float lockedFactor  = evCompFactor(lastEvComp, evStep);
        const float currentFactor = evCompFactor(meta.aeExposureCompensation, evStep);
        const float biasedTarget  = filteredTotalUs * (currentFactor / lockedFactor);

        /* EMA toward the new biased target. First locked frame
         * after unlock (sentinel <= 0) seeds without smoothing so
         * locking from a static auto state is instant; subsequent
         * EV moves get aeDamping × delta per frame, matching the
         * auto path's LPF and avoiding a one-frame exposure step
         * that mid-frame readout would slice apart. */
        if (lockedBiasedTotalUs <= 0.f) {
            lockedBiasedTotalUs = biasedTarget;
        } else {
            lockedBiasedTotalUs = damping * biasedTarget
                                + (1.0f - damping) * lockedBiasedTotalUs;
        }

        fillAeBatch(&batch, sensorCfg, sensorCfg.splitExposure(lockedBiasedTotalUs, limits, flickerLines));
        return out;
    }

    /* Falling through means lock isn't active on this frame —
     * drop the locked-bias EMA so the next lock entry seeds from
     * a fresh target rather than ramping from a stale value. */
    lockedBiasedTotalUs = 0.f;

    /* Spot-meter patch mean over rgbMean.G inside meta.meteringRoi.
     * Saturated patches still pull the metric toward 1.0 (and AE
     * backs off) via their own clipped patch mean; black patches
     * still pull toward 0 via the same mechanism. */
    float meanLuma = meanLumaInRoi(stats, meta);
    if (meanLuma < lumaNoiseFloor) meanLuma = lumaNoiseFloor;

    /* EMA the measured luma before the controller sees it. Static
     * scene + hand-shake + sensor noise would otherwise wiggle raw
     * meanLuma ±30 % frame to frame, which is more than the
     * dead-band is wide. Seed on first hit so the filter doesn't
     * spend its start-up ramp dragging from zero. */
    if (appliedTotalUs > 0.f) {
        /* Smooth the SCENE, not the image: luma divided by the
         * exposure it was taken at. Image luma moves with our own
         * exposure changes, and averaging frames taken at different
         * exposures blends the old operating point into the new
         * measurement. On a static exposure this is the same filter
         * as before. */
        const float scene = meanLuma / appliedTotalUs;
        if (smoothedScene <= 0.f) smoothedScene = scene;
        else                      smoothedScene = damping * scene
                                                + (1.0f - damping) * smoothedScene;
        meanLuma = smoothedScene * appliedTotalUs;
    } else {
        if (smoothedLuma <= 0.f) smoothedLuma = meanLuma;
        else                     smoothedLuma = damping * meanLuma
                                               + (1.0f - damping) * smoothedLuma;
        meanLuma = smoothedLuma;
    }

    /* Apply the framework's exposure compensation (1/3-stop units)
     * by shifting the AE target. evComp == 0 → factor 1.0 → identical
     * to the unbiased setpoint. */
    const float effectiveSetpoint = aeSetpoint
                                  * evCompFactor(meta.aeExposureCompensation, evStep);

    /* Two-candidate AE — mean target + highlight protection,
     * strictest wins (RPi / libcamera convention). Cap = 0.8: top
     * 2% of patches sit at 80% of dynamic range, ~0.3-stop headroom
     * before sensor clip. */
    /* The ratios are not clamped. MaxFstopDelta{Pos,Neg} limit how
     * fast the COMMAND may move per frame (NVIDIA: "limit the
     * effective fstop change per frame"), applied after the LPF
     * below. Clamping the ratio instead limited each measurement to
     * half a stop, and with the correction based on the measured
     * frame's exposure that became half a stop per ~5-frame control
     * cycle: ~12 s to come back from a flashlight. */
    const float ratioMean = effectiveSetpoint / meanLuma;

    constexpr float kAeHighlightCap = 0.8f;
    const float ratioHighlight = (iqmHighlight > 0.f)
                               ? (kAeHighlightCap / iqmHighlight)
                               : ratioMean;

    const float ratio = ratioMean < ratioHighlight ? ratioMean : ratioHighlight;

    /* EV-space target — recomputed every frame from current state
     * and the freshly-measured ratio. Multiplier-space accumulation
     * (the previous design's `state ×= smoothedAeMult`) carried
     * directional inertia past the setpoint crossing, visible as
     * 30-frame overshoot ramps. EV-space target has no directional
     * memory: only the LPF on filteredTotalUs carries history.
     *
     * Envelope clamp [minTotal, maxTotal] doubles as anti-windup. */
    /* The statistics are of the raw frame, before the ISP's digital
     * gain: `appliedTotalUs` leaves it out too, so the target is in
     * the same terms and the digital gain makes up what the analog
     * cannot (RPi AGC: "the statistics reflect the image without
     * digital gain"). The envelope includes it. */
    const float   maxTotal = sensorCfg.maxTotalUs(limits);
    const float   minTotal = (float)sensorCfg.lineTimeNs * 2.0f / 1000.0f;
    /* The correction was measured on a frame exposed at
     * `appliedTotalUs`, so it applies to that, not to whatever the
     * controller has commanded since. Using the latest command here
     * re-applied the same stale correction on every frame of the
     * ~5-frame control latency and overshot — the dark/bright
     * swinging after a sudden scene change. */
    const float base = appliedTotalUs > 0.f ? appliedTotalUs : filteredTotalUs;
    float targetTotalUs = base * ratio;
    if (targetTotalUs < minTotal) targetTotalUs = minTotal;
    if (targetTotalUs > maxTotal) targetTotalUs = maxTotal;

    /* Absolute-deviation dead-band, RPi-style: when target is within
     * ±2 % of filtered state, freeze the filter entirely and report
     * the frame as converged. Sized larger than any single LPF step
     * the controller takes in steady state — kills the limit-cycle
     * pattern from a too-narrow dead-band + control latency. */
    constexpr float kAeStableRegion = 0.02f;
    const float deviation    = targetTotalUs / filteredTotalUs - 1.0f;
    const float absDeviation = deviation < 0.f ? -deviation : deviation;
    if (absDeviation < kAeStableRegion) {
        fillAeBatch(&batch, sensorCfg, sensorCfg.splitExposure(filteredTotalUs, limits, flickerLines));
        if (aeConvergedFrames < INT32_MAX) aeConvergedFrames++;
        lastEvComp = meta.aeExposureCompensation;
        out.converged = (aeConvergedFrames >= kAeConvergedFramesRequired);
        out.luxIndex  = computeLuxIndex(base, meanLuma);
        return out;
    }
    aeConvergedFrames = 0;

    /* Single-pole LPF on absolute target with asymmetric speed when
     * configured: faster pole near the target kills the long
     * settling tail. Per-sensor zone width via
     * active.hal_overrides.ae.close_speed_zone — zero disables the
     * boost (no resonance risk on noisy sensors). */
    const float speed = overFrames((aeCloseSpeedZone > 0.f
                                    && absDeviation < aeCloseSpeedZone)
                                       ? sqrtf(aeDamping)
                                       : aeDamping,
                                   framesElapsed);
    float next = speed * targetTotalUs + (1.0f - speed) * filteredTotalUs;

    /* Slew limit on the command, per elapsed frame. */
    if (aeRatioMax > 1.f) {
        const float up = filteredTotalUs * powf(aeRatioMax, (float)framesElapsed);
        if (next > up) next = up;
    }
    if (aeRatioMin > 0.f && aeRatioMin < 1.f) {
        const float down = filteredTotalUs * powf(aeRatioMin, (float)framesElapsed);
        if (next < down) next = down;
    }
    filteredTotalUs = next;
    lastEvComp = meta.aeExposureCompensation;

    fillAeBatch(&batch, sensorCfg, sensorCfg.splitExposure(filteredTotalUs, limits, flickerLines));
    /* The luma is the raw frame's: the lux is of the exposure it was
     * taken at, without the digital gain. */
    out.luxIndex = computeLuxIndex(base, meanLuma);
    return out;
}

} /* namespace android */
