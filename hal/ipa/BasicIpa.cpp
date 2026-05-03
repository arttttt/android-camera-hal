#include "BasicIpa.h"

#include <math.h>
#include <stdint.h>

#include <system/camera_metadata.h>

#include "3a/AeResult.h"
#include "3a/AfResult.h"
#include "3a/AutoWhiteBalanceController.h"
#include "3a/AwbResult.h"
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
 * scene saturated AE state into phantom territory and the
 * controller needed dozens of frames to unwind once the scene
 * brightened — visible as daylight over-exposure + brightness
 * pumping. */

/* Knob derivations from the sensor's tuning. No silent fallbacks —
 * if a field is absent or zero, the corresponding BasicIpa member
 * ends up at zero, which naturally disables the matching AE branch
 * (aeDamping == 0 → AE freezes, gated on flagged control modes).
 * The contract is that `active.ae` is always populated for any
 * shipping tuning; if it isn't, the IPA does nothing and V4L2 runs
 * on the framework / manual path instead of sneaking in compile-
 * time heuristics. */

float awbSceneLightFloorOf(const SensorTuning *t) {
    if (!t) return 0.f;
    const float v = t->awbParams().cStatsDarkThreshold;
    return v > 0.f ? v : 0.f;
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
 * BasicIpa disables the asymmetric boost), per the project's
 * "no silent fallbacks for 3A tuning knobs" rule. */
float deriveAeCloseSpeedZone(const SensorTuning *t) {
    if (!t) return 0.f;
    return t->aeParams().closeSpeedZone;
}

unsigned toQ8(float x) {
    return (unsigned)(x * 256.0f + 0.5f);
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
 * those patches — the response to focus is unambiguous; the weighted
 * 2x-vs-1x form we tried first averaged outside patches in heavily
 * enough that a small tap shifted the metric by single-digit
 * percent and the AE response read as "didn't follow focus".
 *
 * One helper, two consumers (AE controller and the AWB-gate light
 * floor) so the gate is calibrated against the same number AE
 * actually optimises. */
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
 * the post-WB domain. Used as the "are we about to blow highlights"
 * signal driving the AE highlight-protection candidate.
 *
 * Why ROI-scoped, not whole-frame: AE mean uses the same ROI (spot
 * meter on tap, default centre 8×8 otherwise). Splitting metering
 * region between mean and highlight signals is incoherent — a tap
 * on a dim subject would have AE mean trying to brighten the tap
 * region while the global-frame highlight constraint (driven by an
 * unrelated bright corner) pulled exposure back the other way, and
 * the tap subject would never reach setpoint. Both signals draw
 * from the same patches by construction.
 *
 * Why post-WB max, not raw green: stats live pre-WB, but downstream
 * the demosaic shader applies WB gains (R, B typically 1.5-2.0×, G
 * pinned at 1.0). A patch with raw G=0.5 / B=0.55 already clips on
 * the B channel after a 1.6× WB gain. Picking the channel that
 * clips first (max-of-three after WB) is the honest "what will
 * actually saturate downstream" metric.
 *
 * Why patch grid not pixel histogram: NeonStatsEncoder doesn't
 * produce a histogram today; the patch grid is what we have.
 * Trade-off: we miss specular highlights smaller than one patch
 * (~67×120 px at 1080p), but catch any meaningfully sized bright
 * region. Pixel histogram for specular detection is a separate,
 * larger change in the encoder.
 *
 * topK = max(1, count × 2 / 100). On the default centre 8×8 ROI
 * (64 patches) topK collapses to 1 — IQM degenerates to single-max,
 * which is the right behaviour for a small spot: averaging two
 * "brightest" patches from a 64-patch tap-region would smear the
 * signal across half the ROI, where one patch is enough to demand
 * "don't let this saturate". On full-frame metering (256 patches)
 * topK = 5, matching RPi's q=0.98..1.0 interquantile spec. */
float top2pcMaxPostWbMean(const IpaStats &stats, const IpaFrameMeta &meta,
                          float wbR, float wbB) {
    int pyLo = meta.focusRoiPyLo < 0                 ? 0                 : meta.focusRoiPyLo;
    int pyHi = meta.focusRoiPyHi > IpaStats::PATCH_Y ? IpaStats::PATCH_Y : meta.focusRoiPyHi;
    int pxLo = meta.focusRoiPxLo < 0                 ? 0                 : meta.focusRoiPxLo;
    int pxHi = meta.focusRoiPxHi > IpaStats::PATCH_X ? IpaStats::PATCH_X : meta.focusRoiPxHi;

    const int count = (pyHi - pyLo) * (pxHi - pxLo);
    if (count <= 0) return 0.f;

    /* Static upper bound for the top buffer: full frame is 256
     * patches, 2 % = 5. */
    constexpr int kMaxTopK = 5;
    int topK = (count * 2) / 100;
    if (topK < 1)        topK = 1;
    if (topK > kMaxTopK) topK = kMaxTopK;

    /* Insertion-sort top-K descending. For K ≤ 5 the inner shifts
     * are negligible; std::sort over the full ROI would be many
     * times the work for the same answer. */
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

} /* namespace */

BasicIpa::BasicIpa(const SensorConfig &cfg, IspPipeline *ispPipeline,
                   const SensorTuning *sensorTuning,
                   const float wbGainPrior[3],
                   int16_t *ccmBufQ10)
    : sensorCfg(cfg),
      isp(ispPipeline),
      tuning(sensorTuning),
      ccmBufferQ10(ccmBufQ10),
      aeSetpoint (deriveAeSetpoint (sensorTuning)),
      aeDamping  (deriveAeDamping  (sensorTuning)),
      aeRatioMin (deriveAeRatioMin (sensorTuning)),
      aeRatioMax (deriveAeRatioMax (sensorTuning)),
      aeCloseSpeedZone(deriveAeCloseSpeedZone(sensorTuning)),
      awbSceneLightFloor(awbSceneLightFloorOf(sensorTuning)),
      awbMinChannel(
          sensorTuning && sensorTuning->awbParams().cStatsMinThreshold > 0.f
          ? sensorTuning->awbParams().cStatsMinThreshold : 0.f),
      filteredTotalUs((float)cfg.exposureDefault * (float)cfg.gainDefault
                      / (float)cfg.gainUnit),
      lastEvComp(0),
      lockedBiasedTotalUs(0.f),
      mAwb(new AutoWhiteBalanceController(sensorTuning, wbGainPrior)),
      smoothedLuma(0.f),
      frameCount(0),
      aeLockHeld(false),
      aeConvergedFrames(0) {

    const bool aeOK  = tuning && tuning->aeParams().loaded
                      && aeSetpoint > 0.f && aeDamping > 0.f
                      && aeRatioMin > 0.f && aeRatioMax > 0.f;
    const bool awbOK = tuning && tuning->awbParams().loaded
                      && awbSceneLightFloor > 0.f;
    ALOGD("3A knobs: aeOK=%d aeSetpoint=%.3f aeDamping=%.3f aeRatio=[%.3f,%.3f] "
          "aeCloseZone=%.3f "
          "awbOK=%d awbSceneLightFloor=%.4f wbPrior=(%.3f,%.3f) "
          "gainMax=%d gainUnit=%d maxExpDef=%d tuningLoaded=%d",
          aeOK ? 1 : 0,
          (double)aeSetpoint, (double)aeDamping,
          (double)aeRatioMin, (double)aeRatioMax,
          (double)aeCloseSpeedZone,
          awbOK ? 1 : 0,
          (double)awbSceneLightFloor,
          (double)mAwb->currentWbR(), (double)mAwb->currentWbB(),
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
        const WbGains seed = mAwb->currentGainsQ8();
        isp->setWbGains(seed.r, seed.g, seed.b);
    }
}

BasicIpa::~BasicIpa() = default;

void BasicIpa::reset() {
    filteredTotalUs = (float)sensorCfg.exposureDefault
                    * (float)sensorCfg.gainDefault
                    / (float)sensorCfg.gainUnit;
    lastEvComp     = 0;
    lockedBiasedTotalUs = 0.f;
    smoothedLuma   = 0.f;
    aeLockHeld          = false;
    aeConvergedFrames = 0;

    mAwb->reset();

    /* Re-seed the shader with the priors so the next session starts
     * from the sensor's calibrated daylight anchor even if the first
     * frame is below the AWB gate. WB goes straight to the ISP, CCM
     * takes the same U→CCT→LERP path the per-frame AWB tick uses so
     * cold-start matches where AWB will later converge. Tunings
     * without awb.v4 skip the CCM reseed and leave whatever Camera
     * loaded at build time. */
    if (isp) {
        const WbGains seed = mAwb->currentGainsQ8();
        isp->setWbGains(seed.r, seed.g, seed.b);
    }
    if (tuning && ccmBufferQ10 && tuning->awbParams().loaded) {
        const float U       = logf(mAwb->currentWbB());
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

    /* AWB — delegate to AutoWhiteBalanceController. The controller
     * is pure math; the gating decision (auto / manual / hold) and
     * routing of the result to the ISP and CCM buffer stay here for
     * step 3 of the refactor and move into Ipa3A in step 6. */
    const float sceneLuma = meanLumaInRoi(stats, meta);
    const bool awbRun = (meta.awbMode == ANDROID_CONTROL_AWB_MODE_AUTO)
                     && (meta.awbLock == ANDROID_CONTROL_AWB_LOCK_OFF)
                     && (isp != nullptr)
                     && !isp->awbLocked()
                     && (sceneLuma >= awbSceneLightFloor);

    AwbResult awbResult;
    if (meta.awbMode == ANDROID_CONTROL_AWB_MODE_OFF
        && meta.manualWbValid && isp != nullptr) {
        /* Manual AWB short-circuit. AWB_MODE == OFF +
         * COLOR_CORRECTION_GAINS in the request → push user-provided
         * gains. CCM stays unchanged (we don't honour
         * COLOR_CORRECTION_TRANSFORM); apps that want fully-manual
         * colour set MODE=TRANSFORM_MATRIX which we don't claim. */
        awbResult = mAwb->applyManualGains(meta.manualWbR,
                                           meta.manualWbG,
                                           meta.manualWbB);
    } else if (awbRun) {
        awbResult = mAwb->process(stats);
    }

    /* Route AWB result. Gains hit the shader directly (zero silicon
     * delay); CCM lands in the shared row-major Q10 buffer the
     * demosaic shader reads by pointer. */
    if (awbResult.gains && isp) {
        isp->setWbGains(awbResult.gains->r,
                        awbResult.gains->g,
                        awbResult.gains->b);
    }
    if (awbResult.ccm && ccmBufferQ10) {
        for (int i = 0; i < 9; ++i) ccmBufferQ10[i] = awbResult.ccm->v[i];
    }

    /* Throttled diagnostic. Reads controller state (lastWb / estCct)
     * for fields the AWB block used to fill inline. */
    const bool logTick = ((frameCount++ & 0x1f) == 0u);  /* every 32 frames */
    const int  diagEstCct  = mAwb->currentEstCct();
    const int  diagNValid  = awbResult.validPatchCount;

    /* IQM of the top 2% post-WB max-of-channels inside the AE focus
     * ROI — driver of the AE highlight-protection candidate.
     * Computed here so it lands in the diagnostic log alongside
     * lastWb (the WB gains it depends on) and so the auto-AE branch
     * can reuse it without a second pass over rgbMean. ROI matches
     * meanLumaInRoi so the two AE signals draw from the same set of
     * patches — no metering-region split between mean and highlight. */
    const float iqmHighlight = top2pcMaxPostWbMean(stats, meta,
                                                  mAwb->currentWbR(),
                                                  mAwb->currentWbB());

    if (logTick) {
        int32_t diagExp = 0, diagExtraQ8 = 256;
        sensorCfg.splitExposureGain((int32_t)(filteredTotalUs + 0.5f),
                                     &diagExp, &diagExtraQ8);
        const int32_t diagGain = (int32_t)(((int64_t)sensorCfg.gainUnit
                                           * diagExtraQ8 + 128) / 256);
        const int32_t diagGainClamped =
            diagGain > sensorCfg.gainMax ? sensorCfg.gainMax
                                          : (diagGain < 1 ? 1 : diagGain);
        const WbGains gQ8 = mAwb->currentGainsQ8();
        ALOGD("3A: frame=%u luma=%.3f iqmHi=%.3f nValid=%d awbRun=%d "
              "lastWb=(%.3f,%.3f) Q8=(%u,%u) estCct=%d "
              "totalUs=%.0f exp=%d gain=%d gainClamp=%d evComp=%d",
              frameCount, (double)sceneLuma, (double)iqmHighlight,
              diagNValid, awbRun ? 1 : 0,
              (double)mAwb->currentWbR(), (double)mAwb->currentWbB(),
              gQ8.r, gQ8.b,
              diagEstCct, (double)filteredTotalUs, diagExp,
              diagGain, diagGainClamped,
              meta.aeExposureCompensation);
    }

    /* Manual AE hands exposure / gain authority to the framework.
     * ApplySettingsStage writes its values directly and pushes them
     * into DelayedControls itself for result metadata; an IPA push
     * on the same slot would clobber that. Skip the AE math so last*
     * is frozen at its last auto decision — convergence resumes from
     * there on switch-back. */
    if (meta.aeMode == ANDROID_CONTROL_AE_MODE_OFF) {
        lockedBiasedTotalUs = 0.f;
        return batch;
    }

    /* AE locked — by the framework's AE_LOCK or by AF (across a
     * sweep). Hold the converged operating point: keep
     * `filteredTotalUs` untouched, but **re-publish the held
     * exposure / gain into DelayedControls every frame** so
     * ApplySettingsStage sees a populated `pendingWrite` and stays
     * on the converged values. Returning an empty batch here would
     * fall through to ApplySettingsStage's manual path and write
     * the framework's request-side exposure / gain to the sensor,
     * jumping the image off the converged operating point — which
     * is what an "AE lock" must not do.
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
        return batch;
    }

    /* Falling through means lock isn't active on this frame —
     * drop the locked-bias EMA so the next lock entry seeds from
     * a fresh target rather than ramping from a stale value. */
    lockedBiasedTotalUs = 0.f;

    /* Spot-meter patch mean over rgbMean.G inside meta.focusRoi* —
     * see meanLumaInRoi(). Saturated patches still pull the metric
     * toward 1.0 (and AE backs off) via their own clipped patch mean;
     * black patches still pull toward 0 via the same mechanism.
     * Without a tap meta defaults to centre 8×8 so AE meters the
     * centre as a 64-patch average; with a tap AE meters precisely
     * the user's chosen subject. */
    float meanLuma = meanLumaInRoi(stats, meta);
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

    /* Apply the framework's exposure compensation (1/3-stop units) by
     * shifting the AE target. evComp == 0 → factor 1.0 → identical to
     * the unbiased setpoint. AE then chases the shifted target via the
     * existing controller; convergence behaviour stays the same. */
    const float effectiveSetpoint = aeSetpoint
                                  * aeCompFactor(meta.aeExposureCompensation);

    /* Two-candidate AE — mean target + highlight protection,
     * strictest wins (RPi / libcamera convention).
     *
     * ratioMean = setpoint / luma is the open-loop scale factor that
     * would put us exactly at the mean-grey setpoint this frame.
     *
     * ratioHighlight = highlightCap / IQM_top2% is the scale factor
     * that would pull the brightest 2% of patches (post-WB) down to
     * the cap. Active only when scene contains regions brighter
     * than the cap; on a dim scene IQM is small so ratioHighlight
     * lands above ratioMax (clamped) and falls out of min().
     *
     * Both clamped to the tuning's MaxFstopDelta envelope as a
     * per-frame rate limit. min() picks the candidate that asks for
     * less exposure — i.e. highlight constraint can darken below
     * mean target but never push above it.
     *
     * Cap = 0.8: brightest 2% of patches sit at 80% of the dynamic
     * range, leaving ~0.3-stop headroom before sensor clip. RPi
     * default for similar mode. Compile-time constant for now;
     * promote to per-sensor tuning if it needs scene-adaptive
     * shaping later. */
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

    /* Recompute the absolute target every frame from the current
     * filtered state and the freshly-measured ratio, then LPF
     * filteredTotalUs toward it. The previous loop kept state in
     * multiplier-space (state ×= smoothedAeMult, where smoothedAeMult
     * itself was a cascade EMA), which carried directional inertia
     * past the setpoint crossing — visible on dim scenes as gain
     * climbing for ~30 frames after luma had already crossed the
     * setpoint, then over-correcting back through it.
     *
     * EV-space target has no directional memory: each frame's target
     * is derived afresh from the current measurement, only the
     * single-pole LPF on filteredTotalUs carries history. Crossing
     * the setpoint is one frame's sign flip in (target − filtered),
     * nothing accumulated to unwind. Same convergence speed (~τ =
     * 1/aeDamping ≈ 4 frames at ConvergeSpeed = 0.25), no overshoot
     * by construction.
     *
     * State stays in float "µs at unity gain" so per-frame
     * corrections don't vanish into integer truncation; rounded ints
     * across frames used to lose ~1/256 of the signal on gainUnit=1
     * sensors and stall the LPF at its first rounding step. The
     * envelope clamp [minTotal, maxTotal] also serves as anti-windup:
     * once the state hits maxTotal because the scene is darker than
     * the sensor can reach at default frame_length, the next frame's
     * target stays clamped instead of accumulating phantom headroom
     * to unwind on the way back. */
    const int32_t gainUnit = sensorCfg.gainUnit;
    const float   maxTotal = (float)sensorCfg.maxExposureUsDefault()
                           * (float)sensorCfg.gainMax / (float)gainUnit;
    const float   minTotal = (float)sensorCfg.lineTimeUs * 2.0f;
    float targetTotalUs = filteredTotalUs * ratio;
    if (targetTotalUs < minTotal) targetTotalUs = minTotal;
    if (targetTotalUs > maxTotal) targetTotalUs = maxTotal;

    /* Absolute-deviation dead-band, RPi-style: when target is within
     * ±2 % of filtered state, freeze the filter entirely and report
     * the frame as converged. Sized to be larger than any single LPF
     * step the controller would normally take in steady state, which
     * is what kills the "deadband-too-narrow + control latency =
     * auto-oscillation" limit-cycle pattern (the previous stops-based
     * dead-band of 0.05st ≈ 3.5 % was just under one ratio_clamp ×
     * damping = 8 % step, so it acted as a transit zone, not a
     * resting region). 2 % is below sensor noise on a still scene yet
     * above frame-to-frame measurement jitter — AE neither chases
     * noise nor stalls in a half-converged state. */
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
        return batch;
    }
    aeConvergedFrames = 0;

    /* Single-pole LPF on absolute target with RPi-style asymmetric
     * speed: when target is within ±aeCloseSpeedZone of filtered,
     * the LPF runs at √aeDamping (faster final approach pole) to
     * kill the long settling tail an EMA at τ = 1/aeDamping ≈ 4
     * frames otherwise spends crawling the last ±10%. Outside the
     * zone — and on any sensor where the zone is zero / missing in
     * tuning — the LPF runs at the single aeDamping pole everywhere.
     *
     * Per-sensor zone width because the optimal value depends on
     * the sensor's measurement noise floor: too wide a zone on a
     * noisy sensor makes the boosted pole's response time (~τ = 2
     * frames) sit close to the period of measurement noise, which
     * resonates and the controller starts amplifying scene flutter
     * instead of damping it. Configured from
     * active.hal_overrides.ae.close_speed_zone in tuning JSON; zero
     * disables the boost (no resonance risk, slight loss of
     * convergence sharpness). */
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
    return batch;
}

bool BasicIpa::isAeConverged() const {
    /* Five consecutive in-dead-band frames before reporting
     * converged — short enough that AF doesn't wait a noticeable
     * beat after the first stable entry, long enough that a
     * single-frame measurement fluke clearing the gate doesn't
     * advance us prematurely. */
    constexpr int32_t kRequired = 5;
    return aeConvergedFrames >= kRequired;
}

void BasicIpa::setAeLock(bool lock) {
    aeLockHeld = lock;
}

} /* namespace android */
