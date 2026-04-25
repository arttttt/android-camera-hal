#ifndef HAL_IPA_STUB_IPA_H
#define HAL_IPA_STUB_IPA_H

#include "Ipa.h"

namespace android {

/* No-op IPA. Returns an empty batch (has[*] = false) for every
 * processStats() call — DelayedControls leaves every cell untouched,
 * so ApplySettingsStage's own push() remains the sole source of
 * truth. Used while the stats shader and the real 3A math are still
 * being wired, and as the default when buildInfrastructure runs
 * without stats support. */
class StubIpa : public Ipa {
public:
    DelayedControls::Batch processStats(uint32_t inputSequence,
                                        const IpaStats &stats,
                                        const IpaFrameMeta &meta) override;
    void reset() override;
    /* No AE loop here — always reports converged so the AF gate
     * doesn't block on stub builds; lock is a structural no-op. */
    bool isAeConverged() const override { return true; }
    void setAeLock(bool /*lock*/) override {}
};

} /* namespace android */

#endif /* HAL_IPA_STUB_IPA_H */
