#ifndef HAL_IPA_BASIC_IPA_H
#define HAL_IPA_BASIC_IPA_H

#include <stdint.h>
#include <memory>

#include "Ipa.h"

namespace android {

class  AutoExposureController;
class  AutoFocusController;
class  AutoWhiteBalanceController;
class  IspPipeline;
class  SensorTuning;
struct SensorConfig;

/* Real 3A over the raw-Bayer IpaStats that NeonStatsEncoder produces.
 *
 * Coordination layer for the three 3A controllers — owns the AE and
 * AWB instances, runs the lock / mode / scene-floor gating, routes
 * each controller's result to the appropriate backend (DelayedControls
 * for AE, IspPipeline + ccmBufferQ10 for AWB). AF lives in
 * AutoFocusController, accessed by Camera and StatsProcessStage
 * directly for now (folded into this coordinator in step 6).
 *
 * Stats live in raw-Bayer domain (pre WB / CCM / gamma), matching
 * libcamera IPU3 / rkisp1 convention. Runs on PipelineThread via
 * StatsProcessStage. Every call must finish well below one frame
 * budget (< 1 ms on Tegra K1 CPU). */
class BasicIpa : public Ipa {
public:
    /* `wbGainPrior` carries the sensor's per-CCT neutral priors (R, G, B)
     * from SensorTuning. The AWB controller normalises to R/G and B/G
     * internally so the shader-side WB sees the same white. Falls back
     * to unity safely when the tuning has no CCT sets.
     *
     * `tuning` and `ccmBufferQ10` together drive CCT-aware CCM
     * selection: every AWB tick that updates the gains also writes a
     * fresh row-major Q10 CCM into `ccmBufferQ10`, picking / blending
     * between the sensor's calibrated (wbGain, ccMatrix) anchors.
     * The buffer is caller-owned (Camera's mCcmQ10) and the same
     * pointer the ISP has already been handed via setCcm — updating
     * it in place lets the next demosaic submit read fresh
     * coefficients without any extra plumbing. Pass tuning == nullptr
     * to disable the CCT drive. */
    BasicIpa(const SensorConfig &sensorCfg, IspPipeline *isp,
             AutoFocusController *af,
             const SensorTuning *tuning,
             const float wbGainPrior[3],
             int16_t *ccmBufferQ10);
    /* Out-of-line because mAe / mAwb are unique_ptrs to forward-
     * declared types — implicit destructor would need the full
     * definitions here, which we deliberately keep out to break the
     * include cycle between BasicIpa and the controllers it owns. */
    ~BasicIpa() override;

    DelayedControls::Batch processStats(uint32_t inputSequence,
                                        const IpaStats &stats,
                                        const IpaFrameMeta &meta) override;
    void reset() override;
    bool isAeConverged() const override;
    void setAeLock(bool lock) override;

private:
    const SensorConfig  &sensorCfg;
    IspPipeline         *isp;
    AutoFocusController *af;          /* not owned — lifetime ≤ Camera */
    const SensorTuning  *tuning;
    int16_t             *ccmBufferQ10;

    /* Coordinator-side gating threshold — below this scene-luma
     * value the AWB controller is held (its patches are noise-
     * dominated). Pulled from tuning's awb.v4.cStatsDarkThreshold;
     * lifts into Ipa3A in step 6. */
    float   awbSceneLightFloor;

    /* AE controller — owns EV-space LPF, dead-band, asymmetric
     * speed, highlight constraint, AE-LOCK + EV-comp bias. */
    std::unique_ptr<AutoExposureController> mAe;

    /* AWB controller — owns gray-world math, EMA-relax to prior,
     * gate, CCT estimation, CCM LERP. */
    std::unique_ptr<AutoWhiteBalanceController> mAwb;

    /* Frame counter for throttled diagnostic logs. Incremented on
     * every processStats entry; a single ALOGD fires per N frames. */
    uint32_t frameCount;
};

} /* namespace android */

#endif /* HAL_IPA_BASIC_IPA_H */
