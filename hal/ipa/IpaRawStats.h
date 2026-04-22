#ifndef HAL_IPA_RAW_STATS_H
#define HAL_IPA_RAW_STATS_H

#include <stdint.h>

namespace android {

/* Raw GPU-written statistics layout — uint fixed-point sums,
 * converted to the float IpaStats the IPA consumer sees on the CPU
 * side. Produced by the fused demosaic shader: every workgroup
 * accumulates a per-WG local histogram / rgb / sharpness sum, then
 * atomicAdds its partials into this buffer at global scope.
 *
 * Fixed-point scaling (shader ↔ CPU convention):
 *   - lumaHist    : plain uint pixel counts, no scaling. Stored
 *                   per-patch (PATCH_Y × PATCH_X × HIST_BINS); the
 *                   CPU consumer sums across patches when a
 *                   whole-frame histogram is needed. Per-patch
 *                   layout spreads the 8160 WG / frame fan-in
 *                   across 256 × 128 distinct addresses, so the L2
 *                   atomic contention drops from ~8000 contenders
 *                   per bin to ~32.
 *   - rgbSum      : float channel contribution multiplied by 65536
 *                   (Q16.16). At 1920×1080 the worst-case per-patch
 *                   pixel count is 8040 and per-channel value is [0,1],
 *                   so max accumulator = 8040 × 65536 = 527 M — fits
 *                   uint32 with ~8× headroom.
 *   - sharpSum    : Tenengrad (Gx² + Gy²) in float scaled by 256
 *                   (Q24.8). Max per-pixel contribution is ≈32 (Sobel
 *                   coefficients on luma ∈ [0,1]); 8040 × 32 × 256 =
 *                   65 M — fits uint32.
 *
 * Per-patch pixel count is derivable on the host from imgW/imgH +
 * the WG-to-patch "centre-of-WG wins" mapping, so it is not stored
 * in the raw buffer.
 *
 * Layout matches the std430 SSBO the demosaic shader binds at
 * binding = 4. Size 131072 + 3072 + 1024 = 135168 B. */
struct IpaRawStats {
    static const int HIST_BINS = 128;
    static const int PATCH_X   = 16;
    static const int PATCH_Y   = 16;

    uint32_t lumaHist [PATCH_Y][PATCH_X][HIST_BINS];
    uint32_t rgbSum   [PATCH_Y][PATCH_X][3];
    uint32_t sharpSum [PATCH_Y][PATCH_X];
};

} /* namespace android */

#endif /* HAL_IPA_RAW_STATS_H */
