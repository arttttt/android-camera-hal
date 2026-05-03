#ifndef HAL_3A_AUTO_WHITE_BALANCE_CONTROLLER_H
#define HAL_3A_AUTO_WHITE_BALANCE_CONTROLLER_H

#include "AwbResult.h"

namespace android {

class SensorTuning;
struct IpaStats;

/* Gray-world AWB with a confidence gate, EMA-relax to a sensor-
 * calibrated daylight prior, and CCT-driven CCM LERP across the
 * tuning's CcmSet brackets.
 *
 * Pure math: the controller never reaches into the ISP, never pushes
 * to DelayedControls, never calls V4L2. Inputs are stats / metadata
 * arguments; outputs are an AwbResult struct the coordinator routes
 * to backends (`IspPipeline::setWbGains`, `Camera::mCcmQ10`, result
 * metadata builder).
 *
 * Coordinator-side gating:
 *  - Caller invokes `process` only when the framework is in AWB
 *    AUTO mode, AWB is not locked, no AF sweep is in progress, and
 *    scene luma is above the noise floor. The controller doesn't
 *    re-check those conditions; if invoked, it runs.
 *  - For manual AWB (AWB_MODE = OFF + COLOR_CORRECTION_GAINS in the
 *    request), the caller invokes `applyManualGains` instead — that
 *    path bypasses gray-world and CCT estimation entirely, just
 *    snapshots the user's gains as the new last-known state.
 *  - For all other "should be held" cases, the caller doesn't invoke
 *    the controller at all on this tick — the previous gains stay
 *    on the shader from the last successful invocation.
 *
 * State preserved across ticks: the smoothed `lastWb` gains (the
 * EMA target), the last estimated CCT (for diagnostic / metadata),
 * the prior to relax toward when the gate fails.
 *
 * Construction:
 *  - `tuning` (may be null) drives the AWB knobs (CStatsMin/Dark
 *    thresholds, SmoothingWpTrackingFraction, gray-line soft-clamp
 *    LUT, U→CCT polynomial, CCM LERP between CcmSets). When null,
 *    the controller still runs but produces no CCM updates and
 *    falls back to compile-time AWB constants.
 *  - `wbGainPrior[3]` carries the sensor's calibrated R/G/B prior
 *    (typically the daylight CcmSet's wbGain). Stored normalised to
 *    G = 1.0 for direct EMA-relax target. */
class AutoWhiteBalanceController {
public:
    AutoWhiteBalanceController(const SensorTuning *tuning,
                               const float wbGainPrior[3]);

    /* Auto-mode gray-world tick. Returns AwbResult with the updated
     * gains (always populated post-EMA), an optional CCM update
     * (populated when the tuning has the awb.v4 section loaded),
     * the estimated CCT (Kelvin), the AWB state for result metadata,
     * and the valid-patch count for diagnostics. */
    AwbResult process(const IpaStats &stats);

    /* Manual-mode push. Snapshots the framework-provided absolute
     * R / G / B gains as the new internal state (normalised so
     * G = 1.0 in the controller's R/B-relative model) and returns a
     * Result that echoes the gains for the coordinator to push to
     * the shader. CCM is left untouched — manual gains without
     * COLOR_CORRECTION_TRANSFORM in the request keeps the current
     * per-CCT CCM. */
    AwbResult applyManualGains(float rGainAbs, float gGainAbs, float bGainAbs);

    /* Reset to the daylight prior. Called on session boundary
     * (Camera::closeDevice → Ipa::reset). */
    void reset();

    /* Snapshot of the controller's smoothed gains, normalised with
     * G pinned at 1.0. Public so AE's highlight-protection candidate
     * can read the current WB on a frame where AWB didn't run (skip
     * branch — coordinator passes the cached value into AE). */
    float currentWbR() const { return current.wbR; }
    float currentWbB() const { return current.wbB; }

    /* Same gains as currentWbR/B, in the Q8 form the IspPipeline
     * shader-side WB stage consumes. Convenience for the cold-start
     * shader seed and for any caller that wants to push the cached
     * gains without going through process(). */
    WbGains currentGainsQ8() const;

    /* Estimated CCT (Kelvin) from the most recent successful gray-
     * world tick, or the prior's CCT if AWB never ran in this
     * session. For result metadata + diagnostic. */
    int currentEstCct() const { return current.estCct; }

private:
    const SensorTuning *tuning;

    /* Sensor-calibrated daylight neutral, R / B normalised to G.
     * The EMA-relax target when the gate fails. Constant after
     * construction. */
    float wbRPrior;
    float wbBPrior;

    /* Tuning-driven AWB knobs. Resolved at construction; missing
     * keys leave the corresponding knob at zero, which disables the
     * matching path (gate floor 0 = always-pass, damping 0 = freeze
     * EMA). The "no silent fallbacks" rule applies. */
    float awbMinChannel;
    float awbDamping;

    /* Smoothed state that survives between ticks. */
    struct State {
        float wbR;
        float wbB;
        int   estCct;
    };
    State current;
};

} /* namespace android */

#endif /* HAL_3A_AUTO_WHITE_BALANCE_CONTROLLER_H */
