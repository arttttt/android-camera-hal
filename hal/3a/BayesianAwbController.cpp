#include "BayesianAwbController.h"

#include <stdint.h>

#include "ipa/IpaStats.h"
#include "sensor/SensorTuning.h"

namespace android {

namespace {

unsigned toQ8(float x) {
    return (unsigned)(x * 256.0f + 0.5f);
}

} /* namespace */

BayesianAwbController::BayesianAwbController(const SensorTuning *t,
                                              const float wbGainPrior[3])
    : tuning(t),
      wbRPrior(wbGainPrior[1] > 0.02f
               ? wbGainPrior[0] / wbGainPrior[1] : 1.0f),
      wbBPrior(wbGainPrior[1] > 0.02f
               ? wbGainPrior[2] / wbGainPrior[1] : 1.0f),
      current{wbRPrior, wbBPrior, 0} {
}

WbGains BayesianAwbController::currentGainsQ8() const {
    WbGains g;
    g.r = (uint16_t)toQ8(current.wbR);
    g.g = 256;
    g.b = (uint16_t)toQ8(current.wbB);
    return g;
}

void BayesianAwbController::reset() {
    current.wbR    = wbRPrior;
    current.wbB    = wbBPrior;
    current.estCct = 0;
}

AwbResult BayesianAwbController::process(const IpaStats & /*stats*/,
                                          float /*luxIndex*/) {
    /* Skeleton: emit the cold-start prior gains every tick. The
     * real Bayesian estimator (coarseSearch + prior interpolation +
     * deltaLimit clamp + fineSearch + IIR damping) lands across
     * subsequent steps. validPatchCount stays at the default -1
     * sentinel because no patch math has run on this path. */
    AwbResult out;
    WbGains gainsQ8;
    gainsQ8.r = (uint16_t)toQ8(current.wbR);
    gainsQ8.g = 256;
    gainsQ8.b = (uint16_t)toQ8(current.wbB);
    out.gains  = gainsQ8;
    out.estCct = current.estCct;
    return out;
}

AwbResult BayesianAwbController::applyManualGains(
        float rGainAbs, float gGainAbs, float bGainAbs) {
    AwbResult out;
    const float gNorm = gGainAbs > 1e-3f ? gGainAbs : 1.0f;
    current.wbR = rGainAbs / gNorm;
    current.wbB = bGainAbs / gNorm;

    WbGains gainsQ8;
    gainsQ8.r = (uint16_t)toQ8(current.wbR);
    gainsQ8.g = 256;
    gainsQ8.b = (uint16_t)toQ8(current.wbB);
    out.gains  = gainsQ8;
    out.estCct = current.estCct;
    return out;
}

} /* namespace android */
