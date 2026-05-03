#ifndef HAL_3A_AUTO_EXPOSURE_CONTROLLER_H
#define HAL_3A_AUTO_EXPOSURE_CONTROLLER_H

#include <stdint.h>

#include <camera/CameraMetadata.h>

#include "AeResult.h"

namespace android {

class IpaFrameMeta;
class SensorTuning;
struct IpaStats;
struct SensorConfig;

/* Output of `AutoExposureController::parseManualSettings` — the V4L2
 * triple to push for a manual-AE frame (or the auto-AE cold-start
 * fallback path). DelayedControls::Batch only carries EXPOSURE +
 * GAIN; the manual path also needs a frame_length sized for the
 * requested exposure (long shutter has to grow the readout window
 * past the default frame period), so the helper returns all three
 * values in one struct. ApplySettingsStage builds a V4l2Controls
 * write from this. */
struct ExposureWriteValues {
    int32_t frameLen;
    int32_t exposureUs;
    int32_t gain;
};

/* AE controller — open-loop EV-space target plus a single-pole LPF,
 * with two parallel candidate ratios (mean-grey target + top-2% IQM
 * highlight constraint), strictest wins. Owns the smoothed total-
 * exposure state (`filteredTotalUs`), the input-luma EMA, the
 * convergence dead-band counter, and the AE-LOCK held state. See
 * docs/tier3_architecture.md "Ipa3A AE loop" for the full
 * description of the math.
 *
 * Pure math: the controller never reaches into the ISP, never pushes
 * to DelayedControls, never calls V4L2. Inputs are stats / metadata
 * arguments + the current WB gains (for the highlight constraint's
 * post-WB max-of-channels); outputs are an AeResult struct the
 * coordinator routes to backends.
 *
 * Coordinator-side gating:
 *  - Caller invokes `process` only when `meta.aeMode != AE_MODE_OFF`.
 *    Manual AE keeps the framework as the authority — ApplySettingsStage
 *    parses the request via the static `parseManualSettings` helper
 *    on this class and writes V4L2 directly; invoking `process` for
 *    AE_MODE_OFF would push state without any of it landing on the
 *    sensor.
 *  - Caller is also free to skip the call entirely on AF-sweep
 *    frames; the controller's state is preserved across the gap and
 *    AE resumes from the converged operating point on the next call.
 *
 * State preserved across ticks: `filteredTotalUs`, `smoothedLuma`,
 * `lastEvComp` (for AE-LOCK EV-bias), `lockedBiasedTotalUs` (the
 * smoothed locked-bias EMA), `aeLockHeld`, `aeConvergedFrames`. */
class AutoExposureController {
public:
    AutoExposureController(const SensorConfig &cfg,
                           const SensorTuning *tuning);

    /* Auto-mode tick. Computes the new target, runs the dead-band /
     * LPF math, returns batch + state + convergence flag. */
    AeResult process(const IpaStats     &stats,
                     const IpaFrameMeta &meta,
                     float               currentWbR,
                     float               currentWbB);

    /* Drop the smoothed luma + locked-bias EMA so the next session
     * starts from a clean slate. Other state (`filteredTotalUs`,
     * lock flag) reseeds from sensor defaults the same way the
     * previous Ipa3A::reset did. */
    void reset();

    /* True when the controller has been within the dead-band for
     * `kAeConvergedFramesRequired` consecutive frames. AF reads
     * this to gate continuous-mode retriggers. */
    bool isConverged() const;

    /* Toggle "hold the converged AE target" mode. AF holds this on
     * across a sweep so the score curve isn't distorted by AE
     * chasing brightness mid-scan. */
    void setLock(bool lock);

    /* For diagnostic logging — current EV-space state. */
    float currentFilteredTotalUs() const { return filteredTotalUs; }

    /* Manual-AE / cold-start parsing. Stateless deterministic mapping
     * from request metadata to the V4L2 triple. Reads
     * ANDROID_SENSOR_EXPOSURE_TIME, ANDROID_SENSOR_SENSITIVITY,
     * ANDROID_CONTROL_AE_EXPOSURE_COMPENSATION, ANDROID_CONTROL_AE_MODE
     * (clamped, EV-comp applied, then split based on AE_MODE: manual
     * grows frame_length to fit a long shutter, auto-cold-start
     * splits via SensorConfig::splitExposureGain to keep FPS).
     *
     * Static because there is no inter-frame state to preserve — the
     * call is pure. ApplySettingsStage uses it on the RequestThread
     * for both the AE_MODE_OFF path and the auto-mode cold-start
     * fallback when the IPA hasn't pushed yet. */
    static ExposureWriteValues parseManualSettings(const CameraMetadata &cm,
                                                   const SensorConfig   &cfg);

private:
    const SensorConfig &sensorCfg;

    /* Tuning-derived knobs, resolved at construction. No silent
     * fallbacks — missing keys leave the corresponding knob at zero,
     * which disables the matching path (damping == 0 freezes the
     * LPF, ratio_max == 0 means no rate limit allowed which folds
     * the ratio to 0 and AE freezes). The contract is that a
     * shipping tuning has `active.ae` populated. */
    float aeSetpoint;
    float aeDamping;
    float aeRatioMin;
    float aeRatioMax;
    float aeCloseSpeedZone;
    /* Per-patch noise floor — luma below this is clamped before the
     * controller sees it. Same `cStatsMinThreshold` value AWB uses,
     * shared semantics ("sensor noise prevents reliable readings
     * below this"). Pulled from awb.v4 in tuning. */
    float lumaNoiseFloor;

    /* AE state — total exposure at unity gain (µs), absolute EV
     * space. Each frame: target = filteredTotalUs × ratio (clamped),
     * filteredTotalUs ← LPF(target). Float so per-frame corrections
     * don't vanish into integer truncation on a gainUnit=1 sensor. */
    float filteredTotalUs;

    /* Smoothed scene luma — input filter so measurement noise + scene
     * flutter don't push AE in and out of dead-band on every other
     * frame. EMA'd with aeDamping so the controller sees an already-
     * low-passed reading. */
    float smoothedLuma;

    /* EV-comp value in effect when the lock branch last published.
     * Used to scale the locked exposure by `factor(current) /
     * factor(lastEvComp)` so dragging the EV slider while locked
     * moves exposure proportionally. */
    int32_t lastEvComp;

    /* Smoothed EV-biased held exposure — without the EMA, a hard EV
     * step on a locked AE produces a one-frame jump that lands mid-
     * rolling-shutter and stitches old/new halves visibly.
     * Sentinel <= 0 means "reset on the next lock entry, seed from
     * target instead of EMA". */
    float lockedBiasedTotalUs;

    /* AE-LOCK state. While true, the controller publishes the held
     * exposure / gain pair every frame (so DelayedControls never
     * empties the slot) but doesn't update internal state. AF holds
     * this on across a sweep. */
    bool aeLockHeld;

    /* Convergence tracker. Increments every frame the dead-band
     * branch keeps AE at setpoint; resets when AE leaves dead-band.
     * `isConverged()` reports true once the count crosses
     * `kAeConvergedFramesRequired`. */
    int32_t aeConvergedFrames;
};

} /* namespace android */

#endif /* HAL_3A_AUTO_EXPOSURE_CONTROLLER_H */
