#ifndef HAL_3A_BAYESIAN_AWB_CONTROLLER_H
#define HAL_3A_BAYESIAN_AWB_CONTROLLER_H

#include "Awb.h"

namespace android {

class SensorTuning;
struct IpaStats;

/* RPi-style Bayesian AWB controller.
 *
 * Skeleton: emits the cold-start prior gains every tick — no real
 * estimation yet. Real coarseSearch / prior interpolation /
 * fineSearch land across subsequent steps of the AWB-Bayes
 * migration plan (see docs/awb-bayes.md). Until each step is wired
 * and the calibration session has produced the tuning data, the
 * AwbFactory keeps gray-world as the production path; routing here
 * requires an explicit `active.awb.algorithm = "bayes"` *and* a
 * complete `BayesParams` block.
 *
 * Construction mirrors GrayWorldAwbController so the factory can
 * swap impls without re-shaping callers — same `(tuning,
 * wbGainPrior[3])` signature, same R/B prior normalisation against
 * G. The Bayes-specific tuning surface
 * (`SensorTuning::bayesParams()`) is read in later steps; the
 * skeleton stores `tuning` to avoid touching the header again
 * when those reads land. */
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

    /* Sensor-calibrated daylight neutral, R / B normalised to G.
     * Skeleton seeds `current` with these and keeps emitting them.
     * Real algorithm steps will update `current` per tick from the
     * coarseSearch / fineSearch outputs. */
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
