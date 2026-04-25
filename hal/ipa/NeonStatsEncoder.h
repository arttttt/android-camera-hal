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
    /* Patch-grid rectangle the encoder computes the focus signal
     * (Sobel sharpSum + green² denominator) over. Half-open on each
     * axis. Patches outside the rectangle skip the Sobel kernel and
     * the greenSq accumulation entirely; their `sharpSum` /
     * `greenSqSum` slots in the Partial stay at zero, which finalize
     * surfaces as `focusMetric == 0`. Histogram and rgbMean still run
     * full-frame because AE / AWB consume them across all patches. */
    struct FocusRoi {
        int pyLo;
        int pyHi;
        int pxLo;
        int pxHi;
    };

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
        /* Sum of green-pixel² per patch. Forms the denominator of the
         * exposure-invariant `focusMetric` IpaStats field. Accumulated
         * in u64 for the same reason as sharpSum: the inner NEON loop
         * folds two u32x4 vmlal_u16 lanes into u64 per cell. */
        uint64_t greenSqSum[IpaStats::PATCH_Y][IpaStats::PATCH_X];
        uint32_t lumaHist[IpaStats::HIST_BINS];
    };

    NeonStatsEncoder();
    ~NeonStatsEncoder();

    NeonStatsEncoder(const NeonStatsEncoder &)            = delete;
    NeonStatsEncoder &operator=(const NeonStatsEncoder &) = delete;

    /* All-at-once reduce. See class doc for the Partial-based form.
     *
     * bayer       Tightly-packed raw Bayer frame. width * height pixels,
     *             2 bytes/pixel on V4L2_PIX_FMT_S{RGGB,GRBG,GBRG,BGGR}10
     *             (10-bit values stored in 16-bit, low bits valid),
     *             1 byte/pixel on the 8-bit variants.
     * width,
     * height      Pixel dimensions of the Bayer frame.
     * pixFmt      V4L2 fourcc. Selects bit depth and 2×2 CFA phase.
     * blackLevel  Per-sample optical-black bias the sensor encodes in
     *             the raw output. Subtracted from each histogram
     *             sample before binning and from every accumulated
     *             sum at finalize time, matching what the demosaic
     *             shader does downstream. 0 disables the correction
     *             (tunings without an opticalBlack section). */
    void compute(const void     *bayer,
                 unsigned        width,
                 unsigned        height,
                 uint32_t        pixFmt,
                 uint32_t        blackLevel,
                 const FocusRoi &focusRoi,
                 IpaStats       *out) const;

    /* Zero out a Partial so a fresh cycle can begin. */
    static void resetPartial(Partial *partial);

    /* Accumulate patch rows [pyStart, pyEnd) into `partial`. Any number
     * of calls is allowed provided the combined ranges cover
     * [0, PATCH_Y) exactly once before finalize(). `partial` must have
     * been zeroed by resetPartial() before the first call of a cycle;
     * it is mutated in place. `blackLevel` and `focusRoi` must be the
     * same across all calls within one cycle. */
    void computeRange(const void     *bayer,
                      unsigned        width,
                      unsigned        height,
                      uint32_t        pixFmt,
                      uint32_t        blackLevel,
                      const FocusRoi &focusRoi,
                      Partial        *partial,
                      int             pyStart,
                      int             pyEnd) const;

    /* Emit IpaStats from the accumulated Partial. `pixFmt` selects the
     * raw dynamic range (1023 for 10-bit, 255 for 8-bit); `blackLevel`
     * is then subtracted from every per-channel sum (sumCh already
     * holds raw samples as accumulated) and from the range denominator
     * so rgbMean is normalised against (maxRaw - blackLevel) — the
     * actual signal headroom after optical-black correction. */
    static void finalize(const Partial &partial,
                         uint32_t       pixFmt,
                         uint32_t       blackLevel,
                         IpaStats      *out);
};

} /* namespace android */

#endif /* HAL_IPA_NEON_STATS_ENCODER_H */
