#include "NeonStatsEncoder.h"

#include <stdint.h>
#include <string.h>

#include <arm_neon.h>
#include <linux/videodev2.h>

#include "IpaStats.h"
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
                             float          *sharpAccum) {
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
    *sharpAccum += (float)gx * (float)gx + (float)gy * (float)gy;
}

} /* namespace */

NeonStatsEncoder::NeonStatsEncoder()  {}
NeonStatsEncoder::~NeonStatsEncoder() {}

void NeonStatsEncoder::compute(const void *bayer,
                                unsigned    width,
                                unsigned    height,
                                uint32_t    pixFmt,
                                IpaStats   *out) const {
    if (!out) return;
    memset(out, 0, sizeof(*out));
    if (!bayer || width == 0 || height == 0) return;

    const bool     wide     = is10Bit(pixFmt);
    const uint8_t *cfa      = phaseChannel[IspParams::bayerPhaseFromFourcc(pixFmt)];
    const int      maxValue = wide ? 1023 : 255;
    /* 10-bit → 128 bins = shift 3; 8-bit → shift 1. Keeps the bin index
     * independent of the sensor bit depth while preserving full range
     * coverage. */
    const int      histShift = wide ? 3 : 1;

    const uint16_t *p16 = reinterpret_cast<const uint16_t *>(bayer);
    const uint8_t  *p8  = reinterpret_cast<const uint8_t  *>(bayer);

    /* Patch boundaries precomputed once; values are uneven for sizes
     * not divisible by PATCH_{X,Y} (e.g. 1080 / 16 = 67.5 → alternating
     * 67 / 68 row stripes). */
    unsigned bx[IpaStats::PATCH_X + 1];
    unsigned by[IpaStats::PATCH_Y + 1];
    for (int i = 0; i <= IpaStats::PATCH_X; ++i)
        bx[i] = (unsigned)((uint64_t)i * width  / IpaStats::PATCH_X);
    for (int i = 0; i <= IpaStats::PATCH_Y; ++i)
        by[i] = (unsigned)((uint64_t)i * height / IpaStats::PATCH_Y);

    /* Per-channel sums and counts per patch. Counts are tracked because
     * uneven patch geometry and 2×2 CFA phase shifts mean the pixel-
     * count-per-channel per patch is not a constant fraction of patch
     * area. sharpSum holds the Tenengrad accumulator (sum of Gx²+Gy²
     * on the green sub-lattice) — left unnormalised to match IpaStats'
     * documented contract. */
    uint64_t sumCh   [IpaStats::PATCH_Y][IpaStats::PATCH_X][3];
    uint32_t cntCh   [IpaStats::PATCH_Y][IpaStats::PATCH_X][3];
    float    sharpSum[IpaStats::PATCH_Y][IpaStats::PATCH_X];
    memset(sumCh,    0, sizeof(sumCh));
    memset(cntCh,    0, sizeof(cntCh));
    memset(sharpSum, 0, sizeof(sharpSum));

    for (int py = 0; py < IpaStats::PATCH_Y; ++py) {
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
                    /* NEON: 16 Bayer pixels per iteration. vld2q_u16
                     * deinterleaves by column parity, so p.val[0] carries
                     * the eight pixels at even columns of the chunk and
                     * p.val[1] carries the eight at odd columns. The row
                     * parity has already fixed which of those vectors
                     * holds G; the other holds R or B depending on the
                     * 2×2 phase. Pairwise accumulate widens u16→u32 in
                     * register so per-patch sums never overflow. */
                    const unsigned chunks  = (x1 - x0) / 16u;
                    const unsigned simdEnd = x0 + chunks * 16u;

                    uint32x4_t accEven = vdupq_n_u32(0);
                    uint32x4_t accOdd  = vdupq_n_u32(0);

                    for (; x < simdEnd; x += 16u) {
                        uint16x8x2_t p = vld2q_u16(row16 + x);
                        accEven = vpadalq_u16(accEven, p.val[0]);
                        accOdd  = vpadalq_u16(accOdd,  p.val[1]);

                        /* Histogram and Sobel walk the eight G values
                         * out of whichever lane held them. Extracting
                         * to a tiny stack buffer avoids the immediate-
                         * index requirement of vgetq_lane_u16 while
                         * staying in L1. */
                        uint16_t gbuf[8];
                        vst1q_u16(gbuf, gIsEven ? p.val[0] : p.val[1]);
                        const unsigned xBase = gIsEven ? x : x + 1u;
                        for (int i = 0; i < 8; ++i) {
                            const uint32_t v   = gbuf[i];
                            uint32_t       bin = v >> histShift;
                            if (bin > 127u) bin = 127u;
                            out->lumaHist[bin] += 1u;

                            const unsigned xG = xBase + 2u * (unsigned)i;
                            scalarSobelStep(p16, p8, /*wide=*/true,
                                            width, height, xG, y,
                                            &sharpSum[py][px]);
                        }
                    }

                    sumCh[py][px][chanEven] += hsum_u32x4(accEven);
                    sumCh[py][px][chanOdd]  += hsum_u32x4(accOdd);
                    cntCh[py][px][chanEven] += chunks * 8u;
                    cntCh[py][px][chanOdd]  += chunks * 8u;
                }

                /* Scalar tail for the pixels past the NEON block, and
                 * the whole 8-bit fallback path (production negotiates
                 * 10-bit, so 8-bit performance is not a concern). */
                for (; x < x1; ++x) {
                    const unsigned chan = cfa[yParity * 2u + (x & 1u)];
                    const uint32_t v    = wide ? row16[x] : row8[x];

                    sumCh[py][px][chan] += v;
                    cntCh[py][px][chan] += 1u;

                    if (chan == 1u) {
                        uint32_t bin = v >> histShift;
                        if (bin > 127u) bin = 127u;
                        out->lumaHist[bin] += 1u;
                        scalarSobelStep(p16, p8, wide,
                                        width, height, x, y,
                                        &sharpSum[py][px]);
                    }
                }
            }
        }
    }

    const float invMax = 1.0f / (float)maxValue;
    for (int py = 0; py < IpaStats::PATCH_Y; ++py) {
        for (int px = 0; px < IpaStats::PATCH_X; ++px) {
            for (int c = 0; c < 3; ++c) {
                const uint32_t n = cntCh[py][px][c];
                if (n == 0u) continue;
                out->rgbMean[py][px][c] =
                    (float)sumCh[py][px][c] / (float)n * invMax;
            }
            out->sharpness[py][px] = sharpSum[py][px];
        }
    }
}

} /* namespace android */
