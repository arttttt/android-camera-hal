#include "AwbFactory.h"

#include "BayesianAwbController.h"
#include "GrayWorldAwbController.h"
#include "sensor/SensorTuning.h"

#define LOG_TAG "Cam-AwbFactory"
#include <utils/Log.h>

namespace android {

std::unique_ptr<Awb> createAwb(const SensorTuning *tuning,
                                const float wbGainPrior[3]) {
    /* SensorTuning has already validated the bayes branch — the
     * algorithm flag is only `Bayes` if a complete BayesParams
     * block parsed (see SensorTuning::load). The redundant
     * `bayesParams()` check here keeps the factory robust if that
     * upstream invariant ever loosens. */
    if (tuning
     && tuning->awbAlgorithm() == SensorTuning::AwbAlgorithm::Bayes
     && tuning->bayesParams()) {
        ALOGI("createAwb: bayesian AWB selected");
        return std::unique_ptr<Awb>(
            new BayesianAwbController(tuning, wbGainPrior));
    }
    return std::unique_ptr<Awb>(
        new GrayWorldAwbController(tuning, wbGainPrior));
}

} /* namespace android */
