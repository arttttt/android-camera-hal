#ifndef HAL_IPA_STUB_IPA_H
#define HAL_IPA_STUB_IPA_H

#include "Ipa.h"

namespace android {

/* No-op IPA. Returns an empty batch (has[*] = false) for every
 * processStats() call — the exposure target never gets a decision, so
 * every request takes ApplySettingsStage's manual path. Used while the stats shader and the real 3A math are still
 * being wired, and as the default when buildInfrastructure runs
 * without stats support. */
class StubIpa : public Ipa {
public:
    DelayedControls::Batch processStats(const IpaProcessParams &params) override;
    void reset() override;
    /* No AE loop here — always reports converged so the AF gate
     * doesn't block on stub builds; lock is a structural no-op. */
    bool isAeConverged() const override { return true; }
    bool isAwbConverged() const override { return true; }
    void setAeLock(bool /*lock*/) override {}
};

} /* namespace android */

#endif /* HAL_IPA_STUB_IPA_H */
