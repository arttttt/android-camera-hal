#ifndef HAL_IPA_BASIC_IPA_H
#define HAL_IPA_BASIC_IPA_H

#include <stdint.h>
#include <memory>

#include "Ipa.h"

namespace android {

class  AutoWhiteBalanceController;
class  IspPipeline;
class  SensorTuning;
struct SensorConfig;

/* Real 3A over the raw-Bayer IpaStats that NeonStatsEncoder produces.
 *
 *   AE  — mean-luma metric from the green-channel histogram drives
 *         an exposure/gain split via SensorConfig's range, with a
 *         simple EMA for damping so the preview doesn't pump.
 *   AWB — gray-world over rgbMean[16][16][3], with patch-level
 *         clipping of saturated / near-black tiles. Emits Q8 gains
 *         directly into IspPipeline::setWbGains so the demosaic
 *         shader picks them up on the next dispatch (zero silicon
 *         delay — WB lives in the shader, not the sensor).
 *   AF  — AutoFocusController stays the owner; this class supplies
 *         the sharpness grid when that integration lands.
 *
 * Stats live in raw-Bayer domain (pre WB / CCM / gamma), matching
 * libcamera IPU3 / rkisp1 convention; do not conflate with sRGB
 * luminance.
 *
 * Runs on PipelineThread via StatsProcessStage after the submit's
 * fence signals. Every call must finish well below one frame budget
 * (< 1 ms on Tegra K1 CPU). */
class BasicIpa : public Ipa {
public:
    /* `wbGainPrior` carries the sensor's per-CCT neutral priors (R, G, B)
     * from SensorTuning. BasicIpa normalises to R/G and B/G so the
     * shader-side WB, which keeps G at unity, sees the same white. The
     * AWB loop starts here on construction and drifts back here on
     * reset(); gray-world's EMA pulls away from this baseline only as
     * the scene gives it reason to. Falls back to unity safely when
     * the tuning has no CCT sets (all three entries = 1.0).
     *
     * `tuning` and `ccmBufferQ10` together drive CCT-aware CCM
     * selection: every AWB tick that updates the gains also writes a
     * fresh row-major Q10 CCM into `ccmBufferQ10`, picking / blending
     * between the sensor's calibrated (wbGain, ccMatrix) anchors.
     * The buffer is caller-owned (Camera's mCcmQ10) and the same
     * pointer the ISP has already been handed via setCcm — updating
     * it in place lets the next demosaic submit read fresh coefficients
     * without any extra plumbing. Pass tuning == nullptr to disable
     * the CCT drive and keep the initial CCM stable (test / fallback
     * path). */
    BasicIpa(const SensorConfig &sensorCfg, IspPipeline *isp,
             const SensorTuning *tuning,
             const float wbGainPrior[3],
             int16_t *ccmBufferQ10);
    /* Out-of-line because mAwb is a unique_ptr to a forward-declared
     * type — implicit destructor would need the full definition
     * here, which we deliberately keep out to break the include cycle
     * between BasicIpa and the controllers it owns. */
    ~BasicIpa() override;

    DelayedControls::Batch processStats(uint32_t inputSequence,
                                        const IpaStats &stats,
                                        const IpaFrameMeta &meta) override;
    void reset() override;
    bool isAeConverged() const override;
    void setAeLock(bool lock) override;

private:
    const SensorConfig  &sensorCfg;
    IspPipeline         *isp;
    const SensorTuning  *tuning;
    int16_t             *ccmBufferQ10;

    /* AE knobs resolved at construction from SensorTuning. Stored
     * as members so the ctor init-list can use them and the per-
     * frame loop reads them hot — no tuning dereference on the
     * stats path. AWB knobs of the same shape live inside the
     * AutoWhiteBalanceController. */
    float   aeSetpoint;          /* (HigherTarget + LowerTarget) / 2/255 */
    float   aeDamping;           /* ConvergeSpeed                   */
    float   aeRatioMin;          /* 2^-MaxFstopDeltaNeg             */
    float   aeRatioMax;          /* 2^+MaxFstopDeltaPos             */
    float   aeCloseSpeedZone;    /* hal_overrides.ae.close_speed_zone */
    float   awbSceneLightFloor;  /* CStatsDarkThreshold — coordinator
                                    gate, kept here until step 6 lifts
                                    coordination into Ipa3A          */
    float   awbMinChannel;       /* CStatsMinThreshold — used by AE as
                                    a per-patch noise floor; moves
                                    into AutoExposureController in
                                    step 4                           */

    /* AE state — total exposure at unity gain (µs), i.e. the
     * exposureUs × gain / gainUnit scalar in absolute EV space. Each
     * frame computes a fresh `targetTotalUs = filteredTotalUs × ratio`
     * (ratio = setpoint / measuredLuma, clamped) and low-passes
     * filteredTotalUs toward it; the previous loop kept state in
     * multiplier-space (state ×= smoothedAeMult, with smoothedAeMult
     * itself a cascade EMA), which carried directional inertia past
     * the setpoint crossing and integrated as visible overshoot. EV-
     * space target + single LPF removes the inertia by construction —
     * crossing the setpoint just flips ratio's sign for one step and
     * filteredTotalUs immediately starts moving the other way, no
     * memory of the old direction. Float so per-frame corrections
     * don't vanish into integer truncation on a gainUnit=1 sensor.
     * Exposure / gain are split at write-time via
     * SensorConfig::splitExposureGain. */
    float   filteredTotalUs;

    /* The exposure-compensation value (1/3-stop units) that was in
     * effect at the last filteredTotalUs update. Lets the AE-lock
     * branch scale the held exposure when the framework changes EV
     * during the lock — biased = filteredTotalUs × factor(current) /
     * factor(lastEvComp). */
    int32_t  lastEvComp;

    /* Smoothed version of the EV-biased held exposure. Without it,
     * a hard EV step on a locked AE produces a one-frame jump in
     * the exposure value sent to V4L2 — at low exposure that lands
     * mid-rolling-shutter and the resulting frame stitches the old
     * top half with the new bottom half (visible as an apparent
     * left-right shift with a band of the previous frame). The
     * member is EMA'd toward the per-call biased target with the
     * same aeDamping used elsewhere; sentinel <= 0 means "reset on
     * the next lock entry, seed from target instead of EMA". */
    float    lockedBiasedTotalUs;

    /* AWB controller — owns gray-world math, EMA-relax to prior,
     * gate, CCT estimation, and CCM LERP. Lifetime tied to BasicIpa.
     * The controller exposes `currentWbR/B` and `currentEstCct` for
     * AE highlight-constraint and diagnostics; everything else flows
     * through `process` / `applyManualGains` returning AwbResult. */
    std::unique_ptr<AutoWhiteBalanceController> mAwb;

    /* Smoothed scene luma. Measurement noise and frame-to-frame
     * scene flutter would otherwise push AE in and out of the
     * dead-band on every other frame, which was visible as lost AE
     * stability on a nominally static shot. EMA'd with ConvergeSpeed
     * so the controller sees an already-low-passed reading. */
    float   smoothedLuma;

    /* Frame counter for throttled diagnostic logs. Incremented on
     * every processStats entry; a single ALOGD fires per N frames. */
    uint32_t frameCount;

    /* AE-lock state. While `aeLockHeld` is true, the controller
     * skips the proposal step entirely — DelayedControls keeps
     * publishing the last queued exposure / gain pair, the sensor
     * stays at the converged operating point, and on unlock we
     * resume from the same internal state without a discontinuity.
     * AF holds this on across a sweep so the score curve isn't
     * distorted by AE chasing brightness mid-sweep. */
    bool     aeLockHeld;

    /* Convergence tracker. Increments every frame the dead-band
     * branch keeps AE at setpoint; resets when AE leaves dead-band
     * (real adjustment needed). `isAeConverged()` reports true once
     * the count has held above `kAeConvergedFramesRequired` —
     * enough frames that we can trust the current operating point
     * to be stable rather than just one in-tolerance reading. */
    int32_t  aeConvergedFrames;
};

} /* namespace android */

#endif /* HAL_IPA_BASIC_IPA_H */
