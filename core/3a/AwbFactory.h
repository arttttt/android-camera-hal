#ifndef HAL_3A_AWB_FACTORY_H
#define HAL_3A_AWB_FACTORY_H

#include <memory>

namespace android {

class Awb;
class SensorTuning;

/* Construct the AWB controller selected by the per-sensor tuning's
 * `active.awb.algorithm` field. Returns gray-world by default.
 * Bayesian is honoured only when both the algorithm flag is set
 * AND a complete `BayesParams` block is loaded; if either is
 * missing the factory falls back to gray-world (with a warning
 * that the SensorTuning parser already logged on tuning load).
 *
 * `wbGainPrior[3]` is the sensor's calibrated cold-start neutral
 * (R, G, B) from FusionLights[FusionInitLight]; both impls take
 * it as the initial gain anchor. */
std::unique_ptr<Awb> createAwb(const SensorTuning *tuning,
                                const float wbGainPrior[3]);

} /* namespace android */

#endif /* HAL_3A_AWB_FACTORY_H */
