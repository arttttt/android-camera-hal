#include "StubIpa.h"

#include "IpaStats.h"

namespace android {

DelayedControls::Batch StubIpa::processStats(const IpaProcessParams & /*params*/) {
    DelayedControls::Batch empty;
    for (int i = 0; i < DelayedControls::COUNT; ++i) {
        empty.has[i] = false;
        empty.val[i] = 0;
    }
    return empty;
}

void StubIpa::reset() {}

} /* namespace android */
