#ifndef HAL_IPA_H
#define HAL_IPA_H

#include <stdint.h>

#include <camera/CameraMetadata.h>

#include "IpaFrameMeta.h"
#include "sensor/DelayedControls.h"

namespace android {

class PartialEmitter;
struct IpaStats;
struct SensorConfig;

/* Per-frame inputs to Ipa::processStats. Carries the stats / metadata
 * the controllers consume plus the contextual identity (frameNumber,
 * timestamp) and emit channel (PartialEmitter) the IPA tick uses to
 * publish per-controller partial results to the framework before the
 * final buffer-bearing result lands. SensorConfig stays a const ref
 * because gainToIso/maxExposureUs and friends drive the AE-partial
 * metadata derivation.
 *
 * Pass-by-struct because StubIpa and Ipa3A share the signature; an
 * argument list this wide gets unwieldy fast. */
class DelayedControls;

struct IpaProcessParams {
    uint32_t                 inputSequence;
    uint32_t                 frameNumber;
    int64_t                  timestampNs;
    const IpaStats          &stats;
    const IpaFrameMeta      &meta;
    const CameraMetadata    &requestSettings;  /* request metadata, used as
                                                * partial-CameraMetadata seed
                                                * so AE/AWB state echoes pick
                                                * up the request's *_MODE /
                                                * *_LOCK keys                */
    const SensorConfig      &sensorCfg;
    DelayedControls         *delayedControls;  /* applyControls(frameNumber)
                                                * yields what's physically on
                                                * the sensor right now (used
                                                * to fill the AE partial's
                                                * exposure / sensitivity
                                                * fields with the real
                                                * applied values rather than
                                                * the IPA's just-decided
                                                * future-frame batch).      */
    PartialEmitter          *emitter;     /* nullable on cold start */
};

/* Image Processing Algorithms — the 3A brain.
 *
 * Consumes per-frame GPU statistics and emits a DelayedControls
 * batch describing what exposure / gain should land on a future
 * frame. In-process, synchronous: invoked by PipelineThread on the
 * frame-fence signal, while the stats buffer is still mapped and
 * hot in L1. Every call must finish in well under one frame budget
 * (target < 1 ms on Tegra K1 CPU) — no blocking I/O, no allocation
 * in the hot path.
 *
 * Framework AE / AWB mode + lock flags are delivered via IpaFrameMeta
 * alongside the stats so the IPA can decide per-frame whether to
 * emit an exposure / gain update, hold the last decision, or skip
 * compute entirely. Controls that land outside DelayedControls
 * (notably WB gains, which apply in the ISP shader with zero silicon
 * delay) are pushed to the matching subsystem from inside the
 * implementation — the return value covers only DelayedControls-
 * backed outputs.
 *
 * Implementations are swapped at buildInfrastructure time (stub
 * during bring-up, basic grey-world / Tenengrad later). Consumers
 * are not allowed to downcast — the interface is the contract. */
class Ipa {
public:
    virtual ~Ipa() {}

    /* Inspects stats for frame `params.inputSequence` and returns
     * the control batch the scheduler should publish. Batch entries
     * with has[i] == false mean "no change for this control on
     * this frame" — DelayedControls leaves the matching cell
     * untouched. The IPA tick may also emit per-controller partial
     * result metadata via `params.emitter` between controller calls
     * — apps that gate UI on AE / AWB state see them earlier than
     * the buffer-bearing final partial. */
    virtual DelayedControls::Batch processStats(const IpaProcessParams &params) = 0;

    /* Drops any internal averaging / peak-tracking state. Called
     * on session boundary (closeDevice) alongside the other
     * per-session resets. */
    virtual void reset() = 0;

    /* True when the AE controller has been within its dead-band for
     * long enough to call exposure / gain converged. Used by the AF
     * controller to gate sweep launches on a stable exposure — a
     * sweep started while AE is still chasing the scene gives a
     * brightness-modulated score curve regardless of the focus
     * metric's exposure invariance. Implementations without an AE
     * loop (StubIpa) always return true so the gate doesn't block
     * AF on builds that do their own framework-side exposure. */
    virtual bool isAeConverged() const = 0;

    /* Toggle "hold the converged AE target" mode. The IPA stops
     * proposing new exposure / gain values for the duration of the
     * lock — DelayedControls keeps re-publishing the last queued
     * batch, the sensor stays put, the IPA's internal state stays
     * frozen so it resumes from the converged operating point on
     * unlock instead of restarting from a stale-mid-update value.
     * AF holds this on across a sweep. */
    virtual void setAeLock(bool lock) = 0;
};

} /* namespace android */

#endif /* HAL_IPA_H */
