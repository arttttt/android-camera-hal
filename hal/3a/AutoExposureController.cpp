#include "AutoExposureController.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include <system/camera_metadata.h>

#include "ipa/IpaFrameMeta.h"
#include "ipa/IpaStats.h"
#include "sensor/DelayedControls.h"
#include "sensor/SensorConfig.h"
#include "sensor/SensorTuning.h"

namespace android {

namespace {

/* EV compensation step: each stop scales exposure by 5/4 up or 4/5
 * down. Matches the ANDROID_CONTROL_AE_COMPENSATION_STEP = 1/3
 * advertised in static characteristics (three 5/4 steps ≈ √2 factor,
 * i.e. one stop). Used by the manual / cold-start helper below; the
 * auto-AE path does its EV compensation against the floating-point
 * setpoint via aeCompFactor() and never touches these. */
constexpr int32_t kEvStepUpNum     = 5;
constexpr int32_t kEvStepUpDenom   = 4;
constexpr int32_t kEvStepDownNum   = 4;
constexpr int32_t kEvStepDownDenom = 5;

inline int32_t applyManualEvComp(int32_t exposureUs, int32_t evComp) {
    if (evComp > 0) {
        for (int i = 0; i < evComp; i++)
            exposureUs = exposureUs * kEvStepUpNum / kEvStepUpDenom;
    } else {
        for (int i = 0; i < -evComp; i++)
            exposureUs = exposureUs * kEvStepDownNum / kEvStepDownDenom;
    }
    return exposureUs;
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

/* Translate the framework's 1/3-stop EV compensation count into a
 * multiplier on the AE target. Zero stays exactly 1.0 so the
 * unmodified path doesn't introduce a powf rounding. */
float aeCompFactor(int32_t evCompUnits) {
    if (evCompUnits == 0) return 1.0f;
    return powf(2.0f, (float)evCompUnits / 3.0f);
}

/* Plain mean of the per-patch green channel from IpaStats::rgbMean
 * over the active focus ROI. Same linear pre-WB / pre-CCM domain as
 * the AE setpoint, so the controller compares like with like.
 *
 * Spot-meter shape: patches outside meta.focusRoi* contribute zero
 * weight. Without a tap meta defaults to the project's centre 8×8
 * (IpaStats::FOCUS_ROI_*) so AE meters the centre as a 64-patch
 * average. On a tap the rectangle moves to the user's chosen subject
 * (5×5 minimum from AutoFocusController) and AE meters precisely
 * those patches. */
float meanLumaInRoi(const IpaStats &stats, const IpaFrameMeta &meta) {
    int pyLo = meta.focusRoiPyLo < 0                 ? 0                 : meta.focusRoiPyLo;
    int pyHi = meta.focusRoiPyHi > IpaStats::PATCH_Y ? IpaStats::PATCH_Y : meta.focusRoiPyHi;
    int pxLo = meta.focusRoiPxLo < 0                 ? 0                 : meta.focusRoiPxLo;
    int pxHi = meta.focusRoiPxHi > IpaStats::PATCH_X ? IpaStats::PATCH_X : meta.focusRoiPxHi;

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
    int pyLo = meta.focusRoiPyLo < 0                 ? 0                 : meta.focusRoiPyLo;
    int pyHi = meta.focusRoiPyHi > IpaStats::PATCH_Y ? IpaStats::PATCH_Y : meta.focusRoiPyHi;
    int pxLo = meta.focusRoiPxLo < 0                 ? 0                 : meta.focusRoiPxLo;
    int pxHi = meta.focusRoiPxHi > IpaStats::PATCH_X ? IpaStats::PATCH_X : meta.focusRoiPxHi;

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

} /* namespace */

AutoExposureController::AutoExposureController(const SensorConfig &cfg,
                                                const SensorTuning *tuning)
    : sensorCfg(cfg),
      aeSetpoint(deriveAeSetpoint(tuning)),
      aeDamping(deriveAeDamping(tuning)),
      aeRatioMin(deriveAeRatioMin(tuning)),
      aeRatioMax(deriveAeRatioMax(tuning)),
      aeCloseSpeedZone(deriveAeCloseSpeedZone(tuning)),
      lumaNoiseFloor(
          tuning && tuning->awbParams().cStatsMinThreshold > 0.f
          ? tuning->awbParams().cStatsMinThreshold : 0.f),
      luxAnchor(tuning ? tuning->aeParams().luxAnchor
                       : std::experimental::optional<LuxAnchor>()),
      filteredTotalUs((float)cfg.exposureDefault * (float)cfg.gainDefault
                      / (float)cfg.gainUnit),
      smoothedLuma(0.f),
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
        const CameraMetadata &cm,
        const SensorConfig   &cfg) {
    int32_t exposureUs = cfg.exposureDefault;
    if (cm.exists(ANDROID_SENSOR_EXPOSURE_TIME)) {
        const int64_t exposureNs =
            *cm.find(ANDROID_SENSOR_EXPOSURE_TIME).data.i64;
        exposureUs = (int32_t)(exposureNs / 1000);
    }

    /* EV compensation applied on top of the requested exposure. */
    if (cm.exists(ANDROID_CONTROL_AE_EXPOSURE_COMPENSATION)) {
        const int32_t evComp =
            *cm.find(ANDROID_CONTROL_AE_EXPOSURE_COMPENSATION).data.i32;
        exposureUs = applyManualEvComp(exposureUs, evComp);
    }

    /* Driver-queried envelope; populateSensorConfigFromDriver fills
     * exposureMin / exposureMax from V4L2_CID_EXPOSURE QUERYCTRL. */
    exposureUs = clampInt(exposureUs, cfg.exposureMin, cfg.exposureMax);

    uint8_t aeMode = ANDROID_CONTROL_AE_MODE_ON;
    if (cm.exists(ANDROID_CONTROL_AE_MODE))
        aeMode = *cm.find(ANDROID_CONTROL_AE_MODE).data.u8;

    int32_t gain = cfg.gainUnit;  /* 1.0x baseline */
    if (cm.exists(ANDROID_SENSOR_SENSITIVITY))
        gain = cfg.isoToGain(*cm.find(ANDROID_SENSOR_SENSITIVITY).data.i32);

    ExposureWriteValues out;
    if (aeMode == ANDROID_CONTROL_AE_MODE_OFF) {
        /* Manual AE: app told us exactly what to do. Honour the
         * requested exposure verbatim; if it doesn't fit the default
         * frame_length, grow frame_length so it does (FPS drops
         * accordingly — that's the user's choice when they set a 1 s
         * shutter). No splitExposureGain, no fps preservation. */
        out.exposureUs = exposureUs;
        out.frameLen   = cfg.frameLenForExposure(exposureUs);
    } else {
        /* Auto AE cold-start fallback (IPA hasn't pushed yet). Hold
         * fps at the default frame_length and trade overflow exposure
         * for extra gain so preview cadence stays smooth. */
        int32_t actualExposureUs;
        int32_t extraGainQ8;
        cfg.splitExposureGain(exposureUs, &actualExposureUs, &extraGainQ8);
        gain = (int32_t)((int64_t)gain * extraGainQ8 / cfg.gainUnit);
        out.exposureUs = actualExposureUs;
        out.frameLen   = cfg.frameLenDefault;
    }

    out.gain = clampInt(gain, cfg.gainMin, cfg.gainMax);
    return out;
}

AeResult AutoExposureController::process(const IpaStats     &stats,
                                          const IpaFrameMeta &meta,
                                          float               currentWbR,
                                          float               currentWbB) {
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
     * exposure / gain into DelayedControls every frame** so
     * ApplySettingsStage sees a populated `pendingWrite` and stays
     * on the converged values. Returning an empty batch here would
     * fall through to ApplySettingsStage's manual path and write
     * the framework's request-side exposure / gain to the sensor,
     * jumping the image off the converged operating point.
     *
     * EV compensation still applies as an additive offset on top of
     * the locked level — `filteredTotalUs` already reflects whatever
     * EV was active at convergence (`lastEvComp`); rescale by the
     * ratio `factor(current) / factor(lastEvComp)` so dragging the
     * EV slider while locked moves exposure proportionally instead
     * of being silently latched. */
    if (meta.aeLock == ANDROID_CONTROL_AE_LOCK_ON || aeLockHeld) {
        const float lockedFactor  = aeCompFactor(lastEvComp);
        const float currentFactor = aeCompFactor(meta.aeExposureCompensation);
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
            lockedBiasedTotalUs = aeDamping * biasedTarget
                                + (1.0f - aeDamping) * lockedBiasedTotalUs;
        }

        int32_t heldExposureUs, heldExtraGainQ8;
        sensorCfg.splitExposureGain((int32_t)(lockedBiasedTotalUs + 0.5f),
                                     &heldExposureUs, &heldExtraGainQ8);
        int32_t heldGain = (int32_t)(((int64_t)sensorCfg.gainUnit
                                     * heldExtraGainQ8 + 128) / 256);
        if (heldGain < 1)                 heldGain = 1;
        if (heldGain > sensorCfg.gainMax) heldGain = sensorCfg.gainMax;
        batch.has[DelayedControls::EXPOSURE] = true;
        batch.val[DelayedControls::EXPOSURE] = heldExposureUs;
        batch.has[DelayedControls::GAIN]     = true;
        batch.val[DelayedControls::GAIN]     = heldGain;
        return out;
    }

    /* Falling through means lock isn't active on this frame —
     * drop the locked-bias EMA so the next lock entry seeds from
     * a fresh target rather than ramping from a stale value. */
    lockedBiasedTotalUs = 0.f;

    /* Spot-meter patch mean over rgbMean.G inside meta.focusRoi.
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
    if (smoothedLuma <= 0.f) smoothedLuma = meanLuma;
    else                     smoothedLuma = aeDamping * meanLuma
                                           + (1.0f - aeDamping) * smoothedLuma;
    meanLuma = smoothedLuma;

    /* Apply the framework's exposure compensation (1/3-stop units)
     * by shifting the AE target. evComp == 0 → factor 1.0 → identical
     * to the unbiased setpoint. */
    const float effectiveSetpoint = aeSetpoint
                                  * aeCompFactor(meta.aeExposureCompensation);

    /* Two-candidate AE — mean target + highlight protection,
     * strictest wins (RPi / libcamera convention). Cap = 0.8: top
     * 2% of patches sit at 80% of dynamic range, ~0.3-stop headroom
     * before sensor clip. */
    float ratioMean = effectiveSetpoint / meanLuma;
    if (ratioMean < aeRatioMin) ratioMean = aeRatioMin;
    if (ratioMean > aeRatioMax) ratioMean = aeRatioMax;

    constexpr float kAeHighlightCap = 0.8f;
    float ratioHighlight = (iqmHighlight > 0.f)
                         ? (kAeHighlightCap / iqmHighlight)
                         : aeRatioMax;
    if (ratioHighlight < aeRatioMin) ratioHighlight = aeRatioMin;
    if (ratioHighlight > aeRatioMax) ratioHighlight = aeRatioMax;

    const float ratio = ratioMean < ratioHighlight ? ratioMean : ratioHighlight;

    /* EV-space target — recomputed every frame from current state
     * and the freshly-measured ratio. Multiplier-space accumulation
     * (the previous design's `state ×= smoothedAeMult`) carried
     * directional inertia past the setpoint crossing, visible as
     * 30-frame overshoot ramps. EV-space target has no directional
     * memory: only the LPF on filteredTotalUs carries history.
     *
     * Envelope clamp [minTotal, maxTotal] doubles as anti-windup. */
    const int32_t gainUnit = sensorCfg.gainUnit;
    const float   maxTotal = (float)sensorCfg.maxExposureUsDefault()
                           * (float)sensorCfg.gainMax / (float)gainUnit;
    const float   minTotal = (float)sensorCfg.lineTimeUs * 2.0f;
    float targetTotalUs = filteredTotalUs * ratio;
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
        int32_t heldExposureUs, heldExtraGainQ8;
        sensorCfg.splitExposureGain((int32_t)(filteredTotalUs + 0.5f),
                                     &heldExposureUs, &heldExtraGainQ8);
        int32_t heldGain = (int32_t)(((int64_t)gainUnit
                                     * heldExtraGainQ8 + 128) / 256);
        if (heldGain < 1)                 heldGain = 1;
        if (heldGain > sensorCfg.gainMax) heldGain = sensorCfg.gainMax;
        batch.has[DelayedControls::EXPOSURE] = true;
        batch.val[DelayedControls::EXPOSURE] = heldExposureUs;
        batch.has[DelayedControls::GAIN]     = true;
        batch.val[DelayedControls::GAIN]     = heldGain;
        if (aeConvergedFrames < INT32_MAX) aeConvergedFrames++;
        lastEvComp = meta.aeExposureCompensation;
        out.converged = (aeConvergedFrames >= kAeConvergedFramesRequired);
        out.luxIndex  = computeLuxIndex(filteredTotalUs, meanLuma);
        return out;
    }
    aeConvergedFrames = 0;

    /* Single-pole LPF on absolute target with asymmetric speed when
     * configured: faster pole near the target kills the long
     * settling tail. Per-sensor zone width via
     * active.hal_overrides.ae.close_speed_zone — zero disables the
     * boost (no resonance risk on noisy sensors). */
    const float speed = (aeCloseSpeedZone > 0.f
                      && absDeviation < aeCloseSpeedZone)
                       ? sqrtf(aeDamping)
                       : aeDamping;
    filteredTotalUs = speed * targetTotalUs
                    + (1.0f - speed) * filteredTotalUs;
    lastEvComp = meta.aeExposureCompensation;

    int32_t newExposureUs;
    int32_t newExtraGainQ8;
    sensorCfg.splitExposureGain((int32_t)(filteredTotalUs + 0.5f),
                                 &newExposureUs, &newExtraGainQ8);

    int32_t newGain = (int32_t)(((int64_t)gainUnit * newExtraGainQ8 + 128) / 256);
    if (newGain < 1)                 newGain = 1;
    if (newGain > sensorCfg.gainMax) newGain = sensorCfg.gainMax;

    batch.has[DelayedControls::EXPOSURE] = true;
    batch.val[DelayedControls::EXPOSURE] = newExposureUs;
    batch.has[DelayedControls::GAIN]     = true;
    batch.val[DelayedControls::GAIN]     = newGain;
    out.luxIndex = computeLuxIndex(filteredTotalUs, meanLuma);
    return out;
}

} /* namespace android */
