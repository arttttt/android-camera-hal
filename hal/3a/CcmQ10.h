#ifndef HAL_3A_CCM_Q10_H
#define HAL_3A_CCM_Q10_H

#include <stdint.h>

namespace android {

/* 3×3 colour-correction matrix in Q10 fixed-point (1024 = 1.0).
 * Row-major layout — index `r * 3 + c` is row r, column c.
 *
 * Convention: rows are output channels, columns are input
 * (post-WB-gains) channels — `R_out = v[0]*R + v[1]*G + v[2]*B` etc.
 * NVIDIA `.isp` ships ccMatrix in transposed (column-major) layout;
 * `SensorTuning::ccmForCctLerpQ10` transposes on write so consumers
 * never see the raw layout. See `reference_ccm_convention.md`. */
struct CcmQ10 {
    int16_t v[9];
};

} /* namespace android */

#endif /* HAL_3A_CCM_Q10_H */
