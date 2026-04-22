#ifndef HAL_IPA_BASIC_IPA_H
#define HAL_IPA_BASIC_IPA_H

#include <stdint.h>

#include "Ipa.h"

namespace android {

struct SensorConfig;

/* Real 3A over the raw-Bayer IpaStats that NeonStatsEncoder produces.
 *
 *   AE  — mean-luma metric from the green-channel histogram drives
 *         an exposure/gain split via SensorConfig's range, with a
 *         simple EMA for damping so the preview doesn't pump.
 *   AWB — follow-on commit (gray-world over rgbMean).
 *   AF  — AutoFocusController stays the owner; this class supplies
 *         the sharpness grid when that integration lands.
 *
 * Stats live in raw-Bayer domain (pre WB / CCM / gamma), matching
 * libcamera IPU3 / rkisp1 convention; do not conflate with sRGB
 * luminance.
 *
 * Runs on PipelineThread via StatsProcessStage after the submit's
 * fence signals. Every call must finish well below one frame budget
 * (< 1 ms on Tegra K1 CPU). */
class BasicIpa : public Ipa {
public:
    explicit BasicIpa(const SensorConfig &sensorCfg);

    DelayedControls::Batch processStats(uint32_t inputSequence,
                                        const IpaStats &stats,
                                        const IpaFrameMeta &meta) override;
    void reset() override;

private:
    const SensorConfig &sensorCfg;

    /* AE state. Tracks the last-published decision (not the sensor's
     * actual state). Stays in sync with reality while AE is running
     * in auto mode; after a manual-mode excursion the next
     * processStats re-converges from wherever the scene is now. */
    int32_t lastExposureUs;
    int32_t lastGain;
};

} /* namespace android */

#endif /* HAL_IPA_BASIC_IPA_H */
