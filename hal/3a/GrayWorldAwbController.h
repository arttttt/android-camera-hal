#ifndef HAL_3A_GRAY_WORLD_AWB_CONTROLLER_H
#define HAL_3A_GRAY_WORLD_AWB_CONTROLLER_H

#include "Awb.h"

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
class GrayWorldAwbController : public Awb {
public:
    GrayWorldAwbController(const SensorTuning *tuning,
                           const float wbGainPrior[3]);

    /* Auto-mode gray-world tick. `luxIndex` is ignored — gray-world
     * is brightness-blind; the Bayesian impl is what consumes it. */
    AwbResult process(const IpaStats &stats, float luxIndex) override;

    AwbResult applyManualGains(float rGainAbs, float gGainAbs,
                                float bGainAbs) override;

    void reset() override;

    float    currentWbR()     const override { return current.wbR; }
    float    currentWbB()     const override { return current.wbB; }
    WbGains  currentGainsQ8() const override;
    int      currentEstCct()  const override { return current.estCct; }

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

#endif /* HAL_3A_GRAY_WORLD_AWB_CONTROLLER_H */
