#include "NeonStatsEncoder.h"

#include <stdint.h>
#include <string.h>

#include <arm_neon.h>
#include <linux/videodev2.h>

#include "IspParams.h"

namespace android {

namespace {

/* 2×2 CFA phase table: phaseChannel[phase][(y&1)*2 + (x&1)] gives the
 * sampled channel — 0 = R, 1 = G, 2 = B. Phase ordering matches
 * IspParams::bayerPhaseFromFourcc (0 RGGB, 1 GRBG, 2 GBRG, 3 BGGR). */
const uint8_t phaseChannel[4][4] = {
    { 0, 1, 1, 2 },
    { 1, 0, 2, 1 },
    { 1, 2, 0, 1 },
    { 2, 1, 1, 0 },
};

bool is10Bit(uint32_t pixFmt) {
    return pixFmt == V4L2_PIX_FMT_SRGGB10 || pixFmt == V4L2_PIX_FMT_SGRBG10 ||
           pixFmt == V4L2_PIX_FMT_SGBRG10 || pixFmt == V4L2_PIX_FMT_SBGGR10;
}

/* ARMv7-NEON does not have vaddvq_*, so fold manually. */
inline uint32_t hsum_u32x4(uint32x4_t v) {
    uint32x2_t lo = vget_low_u32(v);
    uint32x2_t hi = vget_high_u32(v);
    uint32x2_t p  = vpadd_u32(lo, hi);
    p             = vpadd_u32(p, p);
    return vget_lane_u32(p, 0);
}

inline void scalarSobelStep(const uint16_t *p16,
                             const uint8_t  *p8,
                             bool            wide,
                             unsigned        width, unsigned height,
                             unsigned        xG, unsigned y,
                             uint64_t       *sharpAccum,
                             uint64_t       *greenSqAccum) {
    const unsigned yN = (y >= 2u)         ? y - 2u : y;
    const unsigned yS = (y + 2u < height) ? y + 2u : y;
    const unsigned xW = (xG >= 2u)            ? xG - 2u : xG;
    const unsigned xE = (xG + 2u < width)     ? xG + 2u : xG;

    uint32_t tl, tc, tr_, ll, rr, bl, bc, br;
    if (wide) {
        tl  = p16[(size_t)yN * width + xW];
        tc  = p16[(size_t)yN * width + xG];
        tr_ = p16[(size_t)yN * width + xE];
        ll  = p16[(size_t)y  * width + xW];
        rr  = p16[(size_t)y  * width + xE];
        bl  = p16[(size_t)yS * width + xW];
        bc  = p16[(size_t)yS * width + xG];
        br  = p16[(size_t)yS * width + xE];
    } else {
        tl  = p8[(size_t)yN * width + xW];
        tc  = p8[(size_t)yN * width + xG];
        tr_ = p8[(size_t)yN * width + xE];
        ll  = p8[(size_t)y  * width + xW];
        rr  = p8[(size_t)y  * width + xE];
        bl  = p8[(size_t)yS * width + xW];
        bc  = p8[(size_t)yS * width + xG];
        br  = p8[(size_t)yS * width + xE];
    }
    const int32_t gx = (int32_t)(tr_ + 2u * rr + br)
                     - (int32_t)(tl  + 2u * ll + bl);
    const int32_t gy = (int32_t)(bl  + 2u * bc + br)
                     - (int32_t)(tl  + 2u * tc + tr_);
    /* |gx|, |gy| ≤ 4·1023 = 4092 on 10-bit Bayer, so gx² + gy² fits
     * comfortably inside int32 (max ≈ 33.5M). Widen to u64 only on
     * the final accumulate. */
    *sharpAccum    += (uint64_t)(gx * gx + gy * gy);
    /* Green pixel² is 1023² ≈ 1M on 10-bit; promote on the way in
     * so we don't sign-extend a high lane out of u32 when the
     * accumulator path goes through s32. */
    const uint32_t gC = wide ? p16[(size_t)y * width + xG]
                              : p8 [(size_t)y * width + xG];
    *greenSqAccum  += (uint64_t)gC * (uint64_t)gC;
}

} /* namespace */

NeonStatsEncoder::NeonStatsEncoder()  {}
NeonStatsEncoder::~NeonStatsEncoder() {}

void NeonStatsEncoder::resetPartial(Partial *partial) {
    memset(partial, 0, sizeof(*partial));
}

void NeonStatsEncoder::computeRange(const void *bayer,
                                     unsigned    width,
                                     unsigned    height,
                                     uint32_t    pixFmt,
                                     uint32_t    blackLevel,
                                     Partial    *partial,
                                     int         pyStart,
                                     int         pyEnd) const {
    if (!partial || !bayer || width == 0 || height == 0) return;
    if (pyStart < 0)                pyStart = 0;
    if (pyEnd   > IpaStats::PATCH_Y) pyEnd  = IpaStats::PATCH_Y;
    if (pyStart >= pyEnd) return;

    const bool     wide     = is10Bit(pixFmt);
    const uint8_t *cfa      = phaseChannel[IspParams::bayerPhaseFromFourcc(pixFmt)];
    /* 10-bit → 128 bins = shift 3; 8-bit → shift 1. Keeps the bin index
     * independent of the sensor bit depth while preserving full range
     * coverage. */
    const int      histShift = wide ? 3 : 1;

    /* Cast once for the per-sample histogram subtract. sumCh keeps
     * the raw accumulation and is corrected in bulk at finalize. */
    const uint32_t bl = blackLevel;

    const uint16_t *p16 = reinterpret_cast<const uint16_t *>(bayer);
    const uint8_t  *p8  = reinterpret_cast<const uint8_t  *>(bayer);

    /* Patch boundaries precomputed once per call; values are uneven for
     * sizes not divisible by PATCH_{X,Y} (e.g. 1080 / 16 = 67.5 →
     * alternating 67 / 68 row stripes). */
    unsigned bx[IpaStats::PATCH_X + 1];
    unsigned by[IpaStats::PATCH_Y + 1];
    for (int i = 0; i <= IpaStats::PATCH_X; ++i)
        bx[i] = (unsigned)((uint64_t)i * width  / IpaStats::PATCH_X);
    for (int i = 0; i <= IpaStats::PATCH_Y; ++i)
        by[i] = (unsigned)((uint64_t)i * height / IpaStats::PATCH_Y);

    for (int py = pyStart; py < pyEnd; ++py) {
        const unsigned y0 = by[py];
        const unsigned y1 = by[py + 1];
        for (unsigned y = y0; y < y1; ++y) {
            const uint32_t yParity = y & 1u;
            const uint8_t  chanEven = cfa[yParity * 2u];
            const uint8_t  chanOdd  = cfa[yParity * 2u + 1u];
            const bool     gIsEven  = (chanEven == 1u);

            const uint16_t *row16 = p16 + (size_t)y * width;
            const uint8_t  *row8  = p8  + (size_t)y * width;

            for (int px = 0; px < IpaStats::PATCH_X; ++px) {
                const unsigned x0 = bx[px];
                const unsigned x1 = bx[px + 1];

                unsigned x = x0;

                if (wide) {
                    const unsigned chunks  = (x1 - x0) / 16u;
                    const unsigned simdEnd = x0 + chunks * 16u;

                    const bool neonSobel =
                        (y >= 2u) && (y + 2u < height)
                        && (x0 >= 2u) && (simdEnd + 2u <= width);

                    const uint16_t *rowN = p16 +
                        (size_t)((y >= 2u) ? y - 2u : y) * width;
                    const uint16_t *rowS = p16 +
                        (size_t)((y + 2u < height) ? y + 2u : y) * width;

                    uint32x4_t accEven   = vdupq_n_u32(0);
                    uint32x4_t accOdd    = vdupq_n_u32(0);
                    /* Sharpness accumulator in int32 lanes. One patch
                     * cell's worst case at 1080p is ~7 chunks × 4 × 16M
                     * per lane ≈ 450M, well inside s32. Folded into
                     * partial->sharpSum (u64) at the end of each cell. */
                    int32x4_t  sharpAccV = vdupq_n_s32(0);
                    /* Sum of green² in u32 lanes — feeds the
                     * exposure-invariant focusMetric. Worst case at
                     * 1080p: ~4000 green pixels per cell × 1023²
                     * spread over 4 lanes ≈ 1G per lane, just inside
                     * u32 (4.3G). Two accumulators (low / high) so
                     * the 8-wide green vector can be squared without
                     * an extra widen. */
                    uint32x4_t greenSqLoV = vdupq_n_u32(0);
                    uint32x4_t greenSqHiV = vdupq_n_u32(0);

                    for (; x < simdEnd; x += 16u) {
                        uint16x8x2_t p = vld2q_u16(row16 + x);
                        accEven = vpadalq_u16(accEven, p.val[0]);
                        accOdd  = vpadalq_u16(accOdd,  p.val[1]);

                        const uint16x8_t gVec = gIsEven ? p.val[0] : p.val[1];

                        /* Square-and-accumulate the green vector. Two
                         * vmlal_u16 (one per 4-lane half) keep the
                         * full u16×u16 → u32 product without widening
                         * outside this NEON kernel. ~4 cycles added
                         * per chunk on Cortex-A15 vs the ~50+ cycles
                         * the rest of the chunk already costs. */
                        greenSqLoV = vmlal_u16(greenSqLoV,
                                                vget_low_u16(gVec),
                                                vget_low_u16(gVec));
                        greenSqHiV = vmlal_u16(greenSqHiV,
                                                vget_high_u16(gVec),
                                                vget_high_u16(gVec));

                        uint16_t gbuf[8];
                        vst1q_u16(gbuf, gVec);
                        for (int i = 0; i < 8; ++i) {
                            uint32_t v = (uint32_t)gbuf[i];
                            v = (v > bl) ? (v - bl) : 0u;
                            partial->lumaHist[v >> histShift] += 1u;
                        }

                        if (neonSobel) {
                            const int lane = gIsEven ? 0 : 1;
                            const uint16x8_t tl = vld2q_u16(rowN + x - 2u).val[lane];
                            const uint16x8_t tc = vld2q_u16(rowN + x     ).val[lane];
                            const uint16x8_t tr = vld2q_u16(rowN + x + 2u).val[lane];
                            const uint16x8_t ll = vld2q_u16(row16 + x - 2u).val[lane];
                            const uint16x8_t rr = vld2q_u16(row16 + x + 2u).val[lane];
                            const uint16x8_t bl = vld2q_u16(rowS + x - 2u).val[lane];
                            const uint16x8_t bc = vld2q_u16(rowS + x     ).val[lane];
                            const uint16x8_t br = vld2q_u16(rowS + x + 2u).val[lane];

                            /* Sobel stays in s16 throughout: max sum is
                             * tr + 2·rr + br = 4·1023 = 4092, well inside
                             * s16. vmull_s16 then widens each squared
                             * result to s32; vmlal_s16 does the fused
                             * square-and-accumulate. Avoids the 16
                             * vmovl_u16 widens and 4 vcvtq_f32_s32s the
                             * s32+float version cost. */
                            const int16x8_t tl_s = vreinterpretq_s16_u16(tl);
                            const int16x8_t tc_s = vreinterpretq_s16_u16(tc);
                            const int16x8_t tr_s = vreinterpretq_s16_u16(tr);
                            const int16x8_t ll_s = vreinterpretq_s16_u16(ll);
                            const int16x8_t rr_s = vreinterpretq_s16_u16(rr);
                            const int16x8_t bl_s = vreinterpretq_s16_u16(bl);
                            const int16x8_t bc_s = vreinterpretq_s16_u16(bc);
                            const int16x8_t br_s = vreinterpretq_s16_u16(br);

                            const int16x8_t gx = vsubq_s16(
                                vaddq_s16(vaddq_s16(tr_s, br_s), vaddq_s16(rr_s, rr_s)),
                                vaddq_s16(vaddq_s16(tl_s, bl_s), vaddq_s16(ll_s, ll_s)));
                            const int16x8_t gy = vsubq_s16(
                                vaddq_s16(vaddq_s16(bl_s, br_s), vaddq_s16(bc_s, bc_s)),
                                vaddq_s16(vaddq_s16(tl_s, tr_s), vaddq_s16(tc_s, tc_s)));

                            sharpAccV = vmlal_s16(sharpAccV, vget_low_s16(gx),  vget_low_s16(gx));
                            sharpAccV = vmlal_s16(sharpAccV, vget_high_s16(gx), vget_high_s16(gx));
                            sharpAccV = vmlal_s16(sharpAccV, vget_low_s16(gy),  vget_low_s16(gy));
                            sharpAccV = vmlal_s16(sharpAccV, vget_high_s16(gy), vget_high_s16(gy));
                        } else {
                            const unsigned xBase = gIsEven ? x : x + 1u;
                            for (int i = 0; i < 8; ++i) {
                                const unsigned xG = xBase + 2u * (unsigned)i;
                                scalarSobelStep(p16, p8, /*wide=*/true,
                                                width, height, xG, y,
                                                &partial->sharpSum[py][px],
                                                &partial->greenSqSum[py][px]);
                            }
                        }
                    }

                    partial->sumCh[py][px][chanEven] += hsum_u32x4(accEven);
                    partial->sumCh[py][px][chanOdd]  += hsum_u32x4(accOdd);
                    partial->cntCh[py][px][chanEven] += chunks * 8u;
                    partial->cntCh[py][px][chanOdd]  += chunks * 8u;
                    partial->sharpSum[py][px]        +=
                        (uint64_t)hsum_u32x4(vreinterpretq_u32_s32(sharpAccV));
                    partial->greenSqSum[py][px]      +=
                        (uint64_t)hsum_u32x4(greenSqLoV)
                      + (uint64_t)hsum_u32x4(greenSqHiV);
                }

                for (; x < x1; ++x) {
                    const unsigned chan = cfa[yParity * 2u + (x & 1u)];
                    const uint32_t v    = wide ? row16[x] : row8[x];

                    partial->sumCh[py][px][chan] += v;
                    partial->cntCh[py][px][chan] += 1u;

                    if (chan == 1u) {
                        const uint32_t vBin = (v > bl) ? (v - bl) : 0u;
                        partial->lumaHist[vBin >> histShift] += 1u;
                        scalarSobelStep(p16, p8, wide,
                                        width, height, x, y,
                                        &partial->sharpSum[py][px],
                                        &partial->greenSqSum[py][px]);
                    }
                }
            }
        }
    }
}

void NeonStatsEncoder::finalize(const Partial &partial,
                                 uint32_t       pixFmt,
                                 uint32_t       blackLevel,
                                 IpaStats      *out) {
    if (!out) return;
    memset(out, 0, sizeof(*out));

    memcpy(out->lumaHist, partial.lumaHist, sizeof(out->lumaHist));

    /* Normalise against the signal range after optical-black, not the
     * raw code range — matches the demosaic shader's rescale by
     * 255 / (maxRaw - blackLevel). A tuning with blackLevel == 0 or
     * a pathological blackLevel >= maxRaw degenerates to the old
     * normalisation so AWB never divides by zero. */
    const uint32_t maxRaw = is10Bit(pixFmt) ? 1023u : 255u;
    const uint32_t denom  = (blackLevel < maxRaw) ? (maxRaw - blackLevel) : 1u;
    const float invMax    = 1.0f / (float)denom;

    for (int py = 0; py < IpaStats::PATCH_Y; ++py) {
        for (int px = 0; px < IpaStats::PATCH_X; ++px) {
            for (int c = 0; c < 3; ++c) {
                const uint32_t n = partial.cntCh[py][px][c];
                if (n == 0u) continue;
                /* Bulk-subtract the optical-black bias: each sample
                 * contributed raw = signal + blackLevel, so the sum
                 * carries `n * blackLevel` of bias. Saturates at 0 on
                 * under-exposed patches where the raw mean sat right
                 * at the black floor. */
                const uint64_t bias = (uint64_t)n * (uint64_t)blackLevel;
                const uint64_t adj  = (partial.sumCh[py][px][c] > bias)
                                      ? partial.sumCh[py][px][c] - bias
                                      : 0ull;
                out->rgbMean[py][px][c] = (float)adj / (float)n * invMax;
            }
            /* Exposure-invariant focus score: gradient² / pixel². The
             * 1.0f floor protects against the all-black patch where
             * greenSqSum is zero — we want a small finite score there,
             * not a divide-by-zero. Per-patch sharpSum peaks around
             * 1.3e11 on extreme high-frequency scenes, well inside
             * float exponent range; ~5 decimal digits of relative
             * precision is fine for AF peak detection, which only
             * compares patches against each other. */
            const uint64_t denomSq = partial.greenSqSum[py][px];
            out->focusMetric[py][px] = (float)partial.sharpSum[py][px]
                                     / (float)(denomSq > 0ull ? denomSq : 1ull);
        }
    }
}

void NeonStatsEncoder::compute(const void *bayer,
                                unsigned    width,
                                unsigned    height,
                                uint32_t    pixFmt,
                                uint32_t    blackLevel,
                                IpaStats   *out) const {
    Partial partial;
    resetPartial(&partial);
    computeRange(bayer, width, height, pixFmt, blackLevel,
                 &partial, 0, IpaStats::PATCH_Y);
    finalize(partial, pixFmt, blackLevel, out);
}

} /* namespace android */
