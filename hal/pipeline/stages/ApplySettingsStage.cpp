#include "ApplySettingsStage.h"

#include "PipelineContext.h"
#include "3a/ExposureControl.h"
#include "3a/AutoFocusController.h"
#include "sensor/SensorConfig.h"

namespace android {

ApplySettingsStage::ApplySettingsStage(const Deps &d) : deps(d) {}

void ApplySettingsStage::process(PipelineContext &ctx) {
    if (deps.sensorCfg) {
        ctx.appliedExposureUs = deps.sensorCfg->exposureDefault;
        ctx.appliedGain       = deps.sensorCfg->gainDefault;
    }

    if (deps.exposure) {
        deps.exposure->onSettings(ctx.request.settings);
        ExposureControl::Report r = deps.exposure->report();
        ctx.appliedExposureUs = r.appliedExposureUs;
        ctx.appliedGain       = r.appliedGain;
    }

    if (deps.af) {
        deps.af->onSettings(ctx.request.settings, ctx.request.frameNumber);
    }
}

} /* namespace android */
