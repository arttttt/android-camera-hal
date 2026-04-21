#ifndef HAL_IPA_STATS_H
#define HAL_IPA_STATS_H

#include <stdint.h>

namespace android {

/* Layout of the host-mapped stats buffer written by the GPU stats
 * compute pass and consumed by the Ipa implementation. Must match
 * the shader binding 1:1 — see isp/vulkan/io/VulkanStatsEncoder.
 *
 * Designed for ~4.5 KB total, one pass over scratchImage, privatized
 * histogram and tree-reduced per-patch sums. Captured once per frame
 * immediately after demosaic, before the per-output blits. */
struct IpaStats {
    static const int HIST_BINS   = 128;
    static const int PATCH_X     = 16;
    static const int PATCH_Y     = 16;

    /* Rec. 601 luma, bin = clamp(luma * HIST_BINS, 0, HIST_BINS-1).
     * uint32_t per bin gives > 4e9 headroom vs 1920*1080 pixels. */
    uint32_t lumaHist[HIST_BINS];

    /* Per-patch mean RGB in [0,1]. Divided by patch pixel count in
     * the shader's tree reduction (not here). */
    float rgbMean[PATCH_Y][PATCH_X][3];

    /* Per-patch Tenengrad sharpness: sum over the patch of
     * (Gx^2 + Gy^2) with 3x3 Sobel on luma. Peak-detect in the AF
     * loop; values are not normalized to pixel count. */
    float sharpness[PATCH_Y][PATCH_X];
};

} /* namespace android */

#endif /* HAL_IPA_STATS_H */
