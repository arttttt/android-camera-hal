#ifndef HAL_IPA_NEON_STATS_ENCODER_H
#define HAL_IPA_NEON_STATS_ENCODER_H

#include <stdint.h>

#include "IpaStats.h"

namespace android {

/* CPU statistics reducer over a raw Bayer slot.
 *
 * Produces the IpaStats layout the IPA consumes, computed from the
 * untouched sensor buffer before WB / CCM / gamma. Raw-space rgbMean
 * and lumaHist match the convention used by libcamera's IPU3 / rkisp1
 * pipelines; IPAs consuming this output must account for the domain
 * difference vs a post-ISP stats path.
 *
 * Two entry points:
 *
 *   compute() — all-at-once. Builds a Partial, runs the full patch
 *               grid, finalises. Convenient for callers that want one
 *               stats result per call in one wall-clock step.
 *
 *   computeRange() / finalize() — progressive. The caller owns a
 *               Partial accumulator, zeroes it with resetPartial(),
 *               invokes computeRange() over any disjoint sequence of
 *               patch-row spans that covers [0, PATCH_Y) exactly once,
 *               then calls finalize() to emit IpaStats. StatsWorker
 *               uses this to spread one statistics computation across
 *               multiple incoming frames so the peak CPU per frame
 *               stays below one sensor period.
 *
 * Stateless — the encoder itself holds no per-frame data; the caller
 * owns any Partial. Thread-safe: multiple StatsWorkers can share one
 * encoder instance. */
class NeonStatsEncoder {
public:
    /* Progressive-compute accumulator. Owned and zeroed by the caller
     * before the first computeRange() call of a cycle; mutated in-place
     * by each subsequent computeRange() and consumed by finalize(). */
    struct Partial {
        uint64_t sumCh   [IpaStats::PATCH_Y][IpaStats::PATCH_X][3];
        uint32_t cntCh   [IpaStats::PATCH_Y][IpaStats::PATCH_X][3];
        /* Tenengrad sum of squared Sobel gradients per patch. Kept as
         * uint64_t so the inner NEON kernel can accumulate in s32 and
         * fold into u64 per patch cell without a float conversion on
         * the hot path — see the vmlal_s16 accumulator in computeRange.
         * finalize() casts down to the IpaStats float field. */
        uint64_t sharpSum[IpaStats::PATCH_Y][IpaStats::PATCH_X];
        uint32_t lumaHist[IpaStats::HIST_BINS];
    };

    NeonStatsEncoder();
    ~NeonStatsEncoder();

    NeonStatsEncoder(const NeonStatsEncoder &)            = delete;
    NeonStatsEncoder &operator=(const NeonStatsEncoder &) = delete;

    /* All-at-once reduce. See class doc for the Partial-based form.
     *
     * bayer    Tightly-packed raw Bayer frame. width * height pixels,
     *          2 bytes/pixel on V4L2_PIX_FMT_S{RGGB,GRBG,GBRG,BGGR}10
     *          (10-bit values stored in 16-bit, low bits valid),
     *          1 byte/pixel on the 8-bit variants.
     * width,
     * height   Pixel dimensions of the Bayer frame.
     * pixFmt   V4L2 fourcc. Selects bit depth and 2×2 CFA phase.
     * out      Receives rgbMean, lumaHist and sharpness. Caller owns
     *          the memory; overwrites it fully. */
    void compute(const void *bayer,
                 unsigned    width,
                 unsigned    height,
                 uint32_t    pixFmt,
                 IpaStats   *out) const;

    /* Zero out a Partial so a fresh cycle can begin. */
    static void resetPartial(Partial *partial);

    /* Accumulate patch rows [pyStart, pyEnd) into `partial`. Any number
     * of calls is allowed provided the combined ranges cover
     * [0, PATCH_Y) exactly once before finalize(). `partial` must have
     * been zeroed by resetPartial() before the first call of a cycle;
     * it is mutated in place. */
    void computeRange(const void *bayer,
                      unsigned    width,
                      unsigned    height,
                      uint32_t    pixFmt,
                      Partial    *partial,
                      int         pyStart,
                      int         pyEnd) const;

    /* Emit IpaStats from the accumulated Partial. pixFmt selects the
     * per-sample dynamic range used to normalise rgbMean to [0, 1]. */
    static void finalize(const Partial &partial,
                         uint32_t       pixFmt,
                         IpaStats      *out);
};

} /* namespace android */

#endif /* HAL_IPA_NEON_STATS_ENCODER_H */
