#ifndef HAL_IPA_BASIC_IPA_H
#define HAL_IPA_BASIC_IPA_H

#include <stdint.h>

#include "Ipa.h"

namespace android {

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

    /* AWB / AE knobs resolved at construction from SensorTuning
     * (with compile-time fallbacks for tunings that predate the
     * active.* promotions). Stored as members so the ctor init-
     * list can use them and the per-frame loop reads them hot —
     * no tuning dereference on the stats path. */
    float   awbMinChannel;       /* CStatsMinThreshold              */
    float   awbSceneLightFloor;  /* CStatsDarkThreshold             */
    float   awbDamping;          /* SmoothingWpTrackingFraction     */
    float   aeSetpoint;          /* (HigherTarget + LowerTarget) / 2/255 */
    float   aeDamping;           /* ConvergeSpeed                   */
    float   aeRatioMin;          /* 2^-MaxFstopDeltaNeg             */
    float   aeRatioMax;          /* 2^+MaxFstopDeltaPos             */
    float   aeToleranceInStops;  /* ToleranceIn — hard dead-band around setpoint */

    /* AE state. Total exposure at unity gain (µs) — the
     * exposureUs × gain / gainUnit scalar the controller accumulates
     * against the setpoint. Stored as float so tiny per-frame
     * corrections (damping × clamped ratio) don't vanish into
     * integer truncation on the gainUnit=1 IMX179 side: rounding
     * exposure and gain back into ints every frame used to trap AE
     * at the first split where extraGainQ8 / 256 dropped the
     * fractional bit, freezing gain at a low step. Exposure and
     * gain are derived at write-time via SensorConfig::splitExposureGain. */
    float   lastTotalUs;

    /* AWB state. R / B gain multipliers relative to G (G pinned at
     * unity on the shader side). The prior values come from the
     * sensor's tuned wbGain at its daylight CcmSet (the hottest-CCT
     * anchor the tuning ships) — the "neutral white on this sensor"
     * point. Floats so the EMA damps cleanly between frames;
     * converted to Q8 only at the setWbGains boundary. */
    float   wbRPrior;
    float   wbBPrior;
    float   lastWbR;
    float   lastWbB;

    /* Smoothed scene luma. Measurement noise and frame-to-frame
     * scene flutter would otherwise push AE in and out of the
     * dead-band on every other frame, which was visible as lost AE
     * stability on a nominally static shot. EMA'd with ConvergeSpeed
     * so the controller sees an already-low-passed reading. */
    float   smoothedLuma;

    /* Second-order smoothing on the AE per-frame multiplier. The
     * first-order damped ratio (1 + (ratio-1)*aeDamping) is run
     * through a second EMA (same ConvergeSpeed) so step scene
     * changes ramp up over a few frames instead of producing a
     * single-frame pop. Now that IMX179 advertises gain in Q8 too
     * (post-kernel-patch), the combined (ratio_clamp × damping ×
     * cascade EMA) chain already lands under perceptibility for
     * every individual frame — no hard per-frame cap needed. */
    float   smoothedAeMult;

    /* Frame counter for throttled diagnostic logs. Incremented on
     * every processStats entry; a single ALOGD fires per N frames. */
    uint32_t frameCount;

    /* AE-lock state. While `mAeLocked` is true, the controller
     * skips the proposal step entirely — DelayedControls keeps
     * publishing the last queued exposure / gain pair, the sensor
     * stays at the converged operating point, and on unlock we
     * resume from the same internal state without a discontinuity.
     * AF holds this on across a sweep so the score curve isn't
     * distorted by AE chasing brightness mid-sweep. */
    bool     mAeLocked;

    /* Convergence tracker. Increments every frame the dead-band
     * branch keeps AE at setpoint; resets when AE leaves dead-band
     * (real adjustment needed). `isAeConverged()` reports true once
     * the count has held above `kAeConvergedFramesRequired` —
     * enough frames that we can trust the current operating point
     * to be stable rather than just one in-tolerance reading. */
    int32_t  mAeConvergedFrames;
};

} /* namespace android */

#endif /* HAL_IPA_BASIC_IPA_H */
