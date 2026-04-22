#include "BasicIpa.h"

#include "IpaStats.h"
#include "sensor/SensorConfig.h"

namespace android {

BasicIpa::BasicIpa(const SensorConfig &cfg) : sensorCfg(cfg) {}

DelayedControls::Batch BasicIpa::processStats(uint32_t /*inputSequence*/,
                                               const IpaStats & /*stats*/) {
    /* Skeleton: no decisions yet. The StubIpa-shaped empty batch keeps
     * the DelayedControls ring untouched so the ApplySettingsStage
     * fallback to request.settings is preserved until the AE math
     * lands in the next commit. */
    DelayedControls::Batch empty;
    for (int i = 0; i < DelayedControls::COUNT; ++i) {
        empty.has[i] = false;
        empty.val[i] = 0;
    }
    return empty;
}

void BasicIpa::reset() {}

} /* namespace android */
