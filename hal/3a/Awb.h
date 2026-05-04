#ifndef HAL_3A_AWB_H
#define HAL_3A_AWB_H

#include "AwbResult.h"
#include "WbGains.h"

namespace android {

struct IpaStats;

/* Per-frame AWB controller interface. Implementations:
 *
 *   - GrayWorldAwbController — patch-grid gray-world + 96-patch
 *     confidence gate + EMA-relax to a sensor-calibrated daylight
 *     prior. Default. Ignores `luxIndex`.
 *   - BayesianAwbController (later step) — lux-conditioned
 *     Bayesian estimator over CT curves with deltaLimit clamp /
 *     bias samples / fineSearch. Selected when
 *     `active.awb.algorithm = "bayes"` AND a complete bayes block
 *     is loaded in the per-sensor tuning.
 *
 * Construction is impl-specific; `AwbFactory::createAwb` consults
 * `SensorTuning::awbAlgorithm()` to pick the variant. Ipa3A is the
 * only consumer.
 *
 * Coordinator-side gating (mode, lock, AF sweep, scene-luma floor)
 * is *outside* this interface. If `process` / `applyManualGains`
 * is called, the impl runs unconditionally — the caller decides
 * whether to invoke at all based on framework state. */
class Awb {
public:
    virtual ~Awb() = default;

    /* Auto-mode tick. `luxIndex` is the AE-derived relative-scale
     * brightness (zero when no `ae.luxAnchor` is calibrated);
     * gray-world ignores it, Bayes consumes it for prior
     * interpolation. */
    virtual AwbResult process(const IpaStats &stats, float luxIndex) = 0;

    /* Manual-mode push (AWB_MODE_OFF + COLOR_CORRECTION_GAINS in
     * the request). Snapshots the framework-provided absolute
     * R / G / B gains as the new internal state and returns a
     * Result that echoes them to the shader. CCM not updated. */
    virtual AwbResult applyManualGains(float rGainAbs,
                                        float gGainAbs,
                                        float bGainAbs) = 0;

    /* Reset to the cold-start prior. Called on session boundary
     * (Camera::closeDevice → Ipa::reset). */
    virtual void reset() = 0;

    /* Snapshot of the controller's smoothed gains, normalised with
     * G pinned at 1.0 — AE's highlight protection reads these on
     * frames AWB didn't run, the throttled diagnostic logs them. */
    virtual float    currentWbR()     const = 0;
    virtual float    currentWbB()     const = 0;
    virtual WbGains  currentGainsQ8() const = 0;
    virtual int      currentEstCct()  const = 0;
};

} /* namespace android */

#endif /* HAL_3A_AWB_H */
