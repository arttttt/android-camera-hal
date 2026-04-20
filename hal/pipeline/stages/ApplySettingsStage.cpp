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

    ExposureControl *exposure = *deps.exposure;
    if (exposure) {
        exposure->onSettings(ctx.request.settings);
        ExposureControl::Report r = exposure->report();
        ctx.appliedExposureUs = r.appliedExposureUs;
        ctx.appliedGain       = r.appliedGain;
    }

    AutoFocusController *af = *deps.af;
    if (af) {
        af->onSettings(ctx.request.settings, ctx.request.frameNumber);
    }
}

} /* namespace android */
