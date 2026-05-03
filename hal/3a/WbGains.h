#ifndef HAL_3A_WB_GAINS_H
#define HAL_3A_WB_GAINS_H

#include <stdint.h>

namespace android {

/* White-balance per-channel gains in Q8 fixed-point (256 = 1.0×).
 * `g` is unity by convention on this HAL (the demosaic shader pins
 * green at 1.0 and applies R / B gains relative to it); carrying it
 * explicitly so the IspPipeline::setWbGains(r, g, b) call site
 * doesn't have to special-case the unit value, and so the same
 * struct is usable by AE's highlight-protection candidate which
 * computes max-of-channels post-WB. */
struct WbGains {
    uint16_t r;
    uint16_t g;
    uint16_t b;
};

} /* namespace android */

#endif /* HAL_3A_WB_GAINS_H */
