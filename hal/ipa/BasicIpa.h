#ifndef HAL_IPA_BASIC_IPA_H
#define HAL_IPA_BASIC_IPA_H

#include <stdint.h>

#include "Ipa.h"

namespace android {

class  IspPipeline;
struct SensorConfig;

/* Real 3A over the raw-Bayer IpaStats that NeonStatsEncoder produces.
 *
 *   AE  — mean-luma metric from the green-channel histogram drives
 *         an exposure/gain split via SensorConfig's range, with a
 *         simple EMA for damping so the preview doesn't pump.
 *   AWB — gray-world over rgbMean[16][16][3], with patch-level
 *         clipping of saturated / near-black tiles. Emits Q8 gains
 *         directly into IspPipeline::setWbGains so the demosaic
 *         shader picks them up on the next dispatch (zero silicon
 *         delay — WB lives in the shader, not the sensor).
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
    BasicIpa(const SensorConfig &sensorCfg, IspPipeline *isp);

    DelayedControls::Batch processStats(uint32_t inputSequence,
                                        const IpaStats &stats,
                                        const IpaFrameMeta &meta) override;
    void reset() override;

private:
    const SensorConfig &sensorCfg;
    IspPipeline        *isp;

    /* AE state. Tracks the last-published decision (not the sensor's
     * actual state). Stays in sync with reality while AE is running
     * in auto mode; after a manual-mode excursion the next
     * processStats re-converges from wherever the scene is now. */
    int32_t lastExposureUs;
    int32_t lastGain;

    /* AWB state. R / B gain multipliers relative to G (which is
     * pinned at unity). Floats so the EMA damps cleanly between
     * frames; converted to Q8 only at the setWbGains boundary. */
    float   lastWbR;
    float   lastWbB;
};

} /* namespace android */

#endif /* HAL_IPA_BASIC_IPA_H */
