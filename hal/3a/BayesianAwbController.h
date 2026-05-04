#ifndef HAL_3A_BAYESIAN_AWB_CONTROLLER_H
#define HAL_3A_BAYESIAN_AWB_CONTROLLER_H

#include <stdint.h>

#include "Awb.h"
#include "sensor/SensorTuning.h"

namespace android {

struct IpaStats;

/* RPi-style Bayesian AWB controller.
 *
 * Two-stage search over the calibrated CT curves
 * (`bayesParams.ctCurveR`, `ctCurveB`):
 *
 *   coarseSearch — log-stepped traversal of the curves' domain
 *     (clipped to a per-mode CT window when the Camera2 awbMode
 *     names a preset like DAYLIGHT / INCANDESCENT). Cost at each
 *     candidate t:
 *       Σ_zones min((gainR·R/G − 1)² + (gainB·B/G − 1)², deltaLimit)
 *         + biasWeight × min((gainR·biasR − 1)² + ..., deltaLimit)
 *         − prior(t | luxIndex)
 *     where (gainR, gainB) = (1/ctR(t), 1/ctB(t)). Picks the
 *     minimising t.
 *
 *   fineSearch — refine the coarseSearch winner along the axis
 *     perpendicular to the CT curve in (R/G, B/G) space, capped
 *     by transversePos / transverseNeg. Adds magenta ↔ green
 *     correction off the Planckian locus (fluorescent banks,
 *     mixed light, sensor-side IR leak).
 *
 * Output-side IIR damping smooths the published gains. First
 * `startupFrames` ticks after reset publish the per-tick estimate
 * verbatim (cold-start snap); afterwards the gains EMA toward
 * the estimate at `damping` rate. The same damping factor pulls
 * `current` toward the prior on no-signal frames so a temporary
 * drop in valid zones doesn't jump the WB.
 *
 * Construction mirrors GrayWorldAwbController so the factory can
 * swap impls without re-shaping callers — same `(tuning,
 * wbGainPrior[3])` signature, same R/B prior normalisation
 * against G as the cold-start anchor and the no-signal fallback. */
class BayesianAwbController : public Awb {
public:
    BayesianAwbController(const SensorTuning *tuning,
                           const float wbGainPrior[3]);

    AwbResult process(const IpaStats &stats,
                       float luxIndex,
                       uint8_t awbMode) override;

    AwbResult applyManualGains(float rGainAbs, float gGainAbs,
                                float bGainAbs) override;

    void reset() override;

    float    currentWbR()     const override { return current.wbR; }
    float    currentWbB()     const override { return current.wbB; }
    WbGains  currentGainsQ8() const override;
    int      currentEstCct()  const override { return current.estCct; }

private:
    const SensorTuning *tuning;

    /* Direct const-pointer into the tuning's optional<BayesParams>;
     * the AwbFactory only constructs us when the optional is engaged,
     * but the pointer-or-null shape lets `process` short-circuit
     * cleanly into the prior fallback if the precondition ever
     * loosens (and avoids re-dereferencing the optional every
     * tick). Lifetime is tied to `tuning` — same camera session. */
    const SensorTuning::BayesParams *bayes;

    /* Sensor-calibrated daylight neutral, R / B normalised to G.
     * Cold-start anchor; also the no-signal EMA-relax target when
     * no zones survive the patch filter on a given tick. */
    float wbRPrior;
    float wbBPrior;

    /* Temporal-smoothing counters. `frameCounter` increments per
     * `process()` call after `reset()`; while it's below
     * `startupFrames` the published gains snap to the per-tick
     * estimate without smoothing. After that the EMA at `damping`
     * rate kicks in. */
    uint32_t frameCounter;

    struct State {
        float wbR;
        float wbB;
        int   estCct;
    };
    State current;
};

} /* namespace android */

#endif /* HAL_3A_BAYESIAN_AWB_CONTROLLER_H */
