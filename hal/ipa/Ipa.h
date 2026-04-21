#ifndef HAL_IPA_H
#define HAL_IPA_H

#include <stdint.h>

#include "sensor/DelayedControls.h"

namespace android {

struct IpaStats;

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
 * Implementations are swapped at buildInfrastructure time (stub
 * during bring-up, basic grey-world / Tenengrad later). Consumers
 * are not allowed to downcast — the interface is the contract. */
class Ipa {
public:
    virtual ~Ipa() {}

    /* Inspects stats for frame `inputSequence` and returns the
     * control batch the scheduler should publish. Batch entries
     * with has[i] == false mean "no change for this control on
     * this frame" — DelayedControls leaves the matching cell
     * untouched. */
    virtual DelayedControls::Batch processStats(uint32_t inputSequence,
                                                const IpaStats &stats) = 0;

    /* Drops any internal averaging / peak-tracking state. Called
     * on session boundary (closeDevice) alongside the other
     * per-session resets. */
    virtual void reset() = 0;
};

} /* namespace android */

#endif /* HAL_IPA_H */
