#ifndef HAL_3A_BAYESIAN_AWB_CONTROLLER_H
#define HAL_3A_BAYESIAN_AWB_CONTROLLER_H

#include "Awb.h"
#include "sensor/SensorTuning.h"

namespace android {

struct IpaStats;

/* RPi-style Bayesian AWB controller.
 *
 * Current state: coarseSearch over the calibrated CT curves
 * (`bayesParams.ctCurveR`, `ctCurveB`). For each candidate CT
 * along a log-stepped traversal of the curves' domain, the
 * controller computes candidate gains `gainR = 1/ctR(t)`,
 * `gainB = 1/ctB(t)` and accumulates per-zone squared error
 * `(gainR·R/G − 1)² + (gainB·B/G − 1)²` over the patches that
 * pass the same saturation / noise-floor filter gray-world uses.
 * The CT minimising the sum is picked. No prior, no off-curve
 * refinement, no temporal smoothing — those land in steps 6, 7,
 * 8 of the migration plan (see docs/awb-bayes.md).
 *
 * Construction mirrors GrayWorldAwbController so the factory can
 * swap impls without re-shaping callers — same `(tuning,
 * wbGainPrior[3])` signature, same R/B prior normalisation
 * against G as the cold-start anchor and the no-signal fallback. */
class BayesianAwbController : public Awb {
public:
    BayesianAwbController(const SensorTuning *tuning,
                           const float wbGainPrior[3]);

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

    /* Direct const-pointer into the tuning's optional<BayesParams>;
     * the AwbFactory only constructs us when the optional is engaged,
     * but the pointer-or-null shape lets `process` short-circuit
     * cleanly into the prior fallback if the precondition ever
     * loosens (and avoids re-dereferencing the optional every
     * tick). Lifetime is tied to `tuning` — same camera session. */
    const SensorTuning::BayesParams *bayes;

    /* Sensor-calibrated daylight neutral, R / B normalised to G.
     * Cold-start anchor; also the no-signal fallback when no zones
     * survive the patch filter on a given tick. */
    float wbRPrior;
    float wbBPrior;

    struct State {
        float wbR;
        float wbB;
        int   estCct;
    };
    State current;
};

} /* namespace android */

#endif /* HAL_3A_BAYESIAN_AWB_CONTROLLER_H */
