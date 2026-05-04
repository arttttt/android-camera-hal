#include "AwbFactory.h"

#include "GrayWorldAwbController.h"
#include "sensor/SensorTuning.h"

#define LOG_TAG "Cam-AwbFactory"
#include <utils/Log.h>

namespace android {

std::unique_ptr<Awb> createAwb(const SensorTuning *tuning,
                                const float wbGainPrior[3]) {
    /* Bayes-impl branch lands in the next step; for now the factory
     * always returns gray-world. Reading `awbAlgorithm()` lets us
     * surface the future-intent miss in a log so misconfigured
     * tunings (algorithm=bayes shipped before BayesianAwbController
     * exists) are visible without silent fallback. */
    if (tuning && tuning->awbAlgorithm() == SensorTuning::AwbAlgorithm::Bayes) {
        ALOGW("createAwb: tuning requested bayes but the impl is not "
              "wired yet; returning gray-world");
    }
    return std::unique_ptr<Awb>(
        new GrayWorldAwbController(tuning, wbGainPrior));
}

} /* namespace android */
