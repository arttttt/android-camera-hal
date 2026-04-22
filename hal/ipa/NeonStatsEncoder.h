#ifndef HAL_IPA_NEON_STATS_ENCODER_H
#define HAL_IPA_NEON_STATS_ENCODER_H

#include <stdint.h>

namespace android {

struct IpaStats;

/* CPU statistics reducer over a raw Bayer slot.
 *
 * Produces the IpaStats layout the IPA consumes, computed from the
 * untouched sensor buffer before WB / CCM / gamma. Raw-space rgbMean
 * and lumaHist match the convention used by libcamera's IPU3 / rkisp1
 * pipelines; IPAs consuming this output must account for the domain
 * difference vs a post-ISP stats path.
 *
 * Stateless. Thread-safe — callers can share one instance across any
 * worker threads that feed it. */
class NeonStatsEncoder {
public:
    NeonStatsEncoder();
    ~NeonStatsEncoder();

    NeonStatsEncoder(const NeonStatsEncoder &)            = delete;
    NeonStatsEncoder &operator=(const NeonStatsEncoder &) = delete;

    /* Reduce a raw Bayer slot into IpaStats.
     *
     * bayer    Tightly-packed raw Bayer frame. width * height pixels,
     *          2 bytes/pixel on V4L2_PIX_FMT_S{RGGB,GRBG,GBRG,BGGR}10
     *          (10-bit values stored in 16-bit, low bits valid),
     *          1 byte/pixel on the 8-bit variants.
     * width,
     * height   Pixel dimensions of the Bayer frame.
     * pixFmt   V4L2 fourcc. Selects bit depth and 2×2 CFA phase.
     * out      Receives rgbMean + lumaHist. sharpness is zero-filled
     *          here; filled in once NEON Sobel lands. Caller owns the
     *          memory; overwrites it fully. */
    void compute(const void *bayer,
                 unsigned    width,
                 unsigned    height,
                 uint32_t    pixFmt,
                 IpaStats   *out) const;
};

} /* namespace android */

#endif /* HAL_IPA_NEON_STATS_ENCODER_H */
