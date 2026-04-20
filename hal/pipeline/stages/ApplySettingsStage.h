#ifndef HAL_PIPELINE_STAGES_APPLY_SETTINGS_STAGE_H
#define HAL_PIPELINE_STAGES_APPLY_SETTINGS_STAGE_H

#include "PipelineStage.h"

namespace android {

class  ExposureControl;
class  AutoFocusController;
struct SensorConfig;

/* Applies per-request exposure + AF settings to the sensor/VCM. Reads
 * context.request.settings, writes context.appliedExposureUs /
 * appliedGain from ExposureControl's report (seeded from sensor
 * defaults when ExposureControl is null, e.g. HW-ISP path). */
class ApplySettingsStage : public PipelineStage {
public:
    struct Deps {
        ExposureControl     **exposure;   /* pointer-to-field; may become null between configures */
        AutoFocusController **af;
        const SensorConfig   *sensorCfg;
    };

    explicit ApplySettingsStage(const Deps &deps);

    const char *name() const override { return "ApplySettings"; }
    void process(PipelineContext &context) override;

private:
    Deps deps;
};

} /* namespace android */

#endif /* HAL_PIPELINE_STAGES_APPLY_SETTINGS_STAGE_H */
