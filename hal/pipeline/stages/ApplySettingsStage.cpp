#include "ApplySettingsStage.h"

#include <system/camera_metadata.h>

#include "PipelineContext.h"
#include "3a/ExposureControl.h"
#include "3a/AutoFocusController.h"
#include "sensor/DelayedControls.h"
#include "sensor/SensorConfig.h"

namespace android {

ApplySettingsStage::ApplySettingsStage(const Deps &d) : deps(d) {}

void ApplySettingsStage::process(PipelineContext &ctx) {
    if (deps.sensorCfg) {
        ctx.appliedExposureUs = deps.sensorCfg->exposureDefault;
        ctx.appliedGain       = deps.sensorCfg->gainDefault;
    }

    if (deps.exposure) {
        uint8_t aeMode = ANDROID_CONTROL_AE_MODE_ON;
        if (ctx.request.settings.exists(ANDROID_CONTROL_AE_MODE)) {
            aeMode = *ctx.request.settings.find(ANDROID_CONTROL_AE_MODE).data.u8;
        }

        bool appliedFromRing = false;
        if (aeMode != ANDROID_CONTROL_AE_MODE_OFF && deps.delayedControls) {
            /* Auto AE — consume the IPA's decision if it has landed
             * for this frame. The ring is empty on cold start and
             * whenever the IPA (e.g. StubIpa) returns empty batches;
             * fall through to the manual path so the sensor keeps a
             * valid exposure / gain rather than freezing. */
            DelayedControls::Batch batch =
                deps.delayedControls->pendingWrite(ctx.request.frameNumber);
            if (batch.has[DelayedControls::EXPOSURE]
             || batch.has[DelayedControls::GAIN]) {
                deps.exposure->applyBatch(batch);
                appliedFromRing = true;
            }
        }

        if (!appliedFromRing) {
            /* Manual AE, or auto-with-no-IPA-push: parse the request
             * and write V4L2 directly. */
            deps.exposure->onSettings(ctx.request.settings);
        }

        ExposureControl::Report r = deps.exposure->report();
        ctx.appliedExposureUs = r.appliedExposureUs;
        ctx.appliedGain       = r.appliedGain;

        /* Publish the value physically written at slot
         * request.frameNumber so ResultMetadataBuilder's
         * applyControls(frame + delay) query returns the same
         * numbers we just wrote. Skipped when the auto path already
         * applied from the ring — IPA's push is already there. */
        if (!appliedFromRing && deps.delayedControls) {
            DelayedControls::Batch published;
            for (int i = 0; i < DelayedControls::COUNT; ++i) {
                published.has[i] = false;
                published.val[i] = 0;
            }
            published.has[DelayedControls::EXPOSURE] = true;
            published.val[DelayedControls::EXPOSURE] = r.appliedExposureUs;
            published.has[DelayedControls::GAIN]     = true;
            published.val[DelayedControls::GAIN]     = r.appliedGain;
            deps.delayedControls->push(ctx.request.frameNumber, published);
        }
    }

    if (deps.af) {
        deps.af->onSettings(ctx.request.settings, ctx.request.frameNumber);
    }
}

} /* namespace android */
