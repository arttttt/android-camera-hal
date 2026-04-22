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

inline float hsum_f32x4(float32x4_t v) {
    float32x2_t lo = vget_low_f32(v);
    float32x2_t hi = vget_high_f32(v);
    float32x2_t p  = vpadd_f32(lo, hi);
    p              = vpadd_f32(p, p);
    return vget_lane_f32(p, 0);
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

void NeonStatsEncoder::resetPartial(Partial *partial) {
    memset(partial, 0, sizeof(*partial));
}

void NeonStatsEncoder::computeRange(const void *bayer,
                                     unsigned    width,
                                     unsigned    height,
                                     uint32_t    pixFmt,
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

                    uint32x4_t  accEven   = vdupq_n_u32(0);
                    uint32x4_t  accOdd    = vdupq_n_u32(0);
                    float32x4_t sharpAccV = vdupq_n_f32(0.0f);

                    for (; x < simdEnd; x += 16u) {
                        uint16x8x2_t p = vld2q_u16(row16 + x);
                        accEven = vpadalq_u16(accEven, p.val[0]);
                        accOdd  = vpadalq_u16(accOdd,  p.val[1]);

                        uint16_t gbuf[8];
                        vst1q_u16(gbuf, gIsEven ? p.val[0] : p.val[1]);
                        for (int i = 0; i < 8; ++i) {
                            uint32_t bin = (uint32_t)gbuf[i] >> histShift;
                            if (bin > 127u) bin = 127u;
                            partial->lumaHist[bin] += 1u;
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

                            const int32x4_t tl_lo = vreinterpretq_s32_u32(vmovl_u16(vget_low_u16(tl)));
                            const int32x4_t tl_hi = vreinterpretq_s32_u32(vmovl_u16(vget_high_u16(tl)));
                            const int32x4_t tc_lo = vreinterpretq_s32_u32(vmovl_u16(vget_low_u16(tc)));
                            const int32x4_t tc_hi = vreinterpretq_s32_u32(vmovl_u16(vget_high_u16(tc)));
                            const int32x4_t tr_lo = vreinterpretq_s32_u32(vmovl_u16(vget_low_u16(tr)));
                            const int32x4_t tr_hi = vreinterpretq_s32_u32(vmovl_u16(vget_high_u16(tr)));
                            const int32x4_t ll_lo = vreinterpretq_s32_u32(vmovl_u16(vget_low_u16(ll)));
                            const int32x4_t ll_hi = vreinterpretq_s32_u32(vmovl_u16(vget_high_u16(ll)));
                            const int32x4_t rr_lo = vreinterpretq_s32_u32(vmovl_u16(vget_low_u16(rr)));
                            const int32x4_t rr_hi = vreinterpretq_s32_u32(vmovl_u16(vget_high_u16(rr)));
                            const int32x4_t bl_lo = vreinterpretq_s32_u32(vmovl_u16(vget_low_u16(bl)));
                            const int32x4_t bl_hi = vreinterpretq_s32_u32(vmovl_u16(vget_high_u16(bl)));
                            const int32x4_t bc_lo = vreinterpretq_s32_u32(vmovl_u16(vget_low_u16(bc)));
                            const int32x4_t bc_hi = vreinterpretq_s32_u32(vmovl_u16(vget_high_u16(bc)));
                            const int32x4_t br_lo = vreinterpretq_s32_u32(vmovl_u16(vget_low_u16(br)));
                            const int32x4_t br_hi = vreinterpretq_s32_u32(vmovl_u16(vget_high_u16(br)));

                            const int32x4_t gx_lo = vsubq_s32(
                                vaddq_s32(vaddq_s32(tr_lo, br_lo), vshlq_n_s32(rr_lo, 1)),
                                vaddq_s32(vaddq_s32(tl_lo, bl_lo), vshlq_n_s32(ll_lo, 1)));
                            const int32x4_t gx_hi = vsubq_s32(
                                vaddq_s32(vaddq_s32(tr_hi, br_hi), vshlq_n_s32(rr_hi, 1)),
                                vaddq_s32(vaddq_s32(tl_hi, bl_hi), vshlq_n_s32(ll_hi, 1)));
                            const int32x4_t gy_lo = vsubq_s32(
                                vaddq_s32(vaddq_s32(bl_lo, br_lo), vshlq_n_s32(bc_lo, 1)),
                                vaddq_s32(vaddq_s32(tl_lo, tr_lo), vshlq_n_s32(tc_lo, 1)));
                            const int32x4_t gy_hi = vsubq_s32(
                                vaddq_s32(vaddq_s32(bl_hi, br_hi), vshlq_n_s32(bc_hi, 1)),
                                vaddq_s32(vaddq_s32(tl_hi, tr_hi), vshlq_n_s32(tc_hi, 1)));

                            const float32x4_t gx_lo_f = vcvtq_f32_s32(gx_lo);
                            const float32x4_t gx_hi_f = vcvtq_f32_s32(gx_hi);
                            const float32x4_t gy_lo_f = vcvtq_f32_s32(gy_lo);
                            const float32x4_t gy_hi_f = vcvtq_f32_s32(gy_hi);

                            sharpAccV = vmlaq_f32(sharpAccV, gx_lo_f, gx_lo_f);
                            sharpAccV = vmlaq_f32(sharpAccV, gx_hi_f, gx_hi_f);
                            sharpAccV = vmlaq_f32(sharpAccV, gy_lo_f, gy_lo_f);
                            sharpAccV = vmlaq_f32(sharpAccV, gy_hi_f, gy_hi_f);
                        } else {
                            const unsigned xBase = gIsEven ? x : x + 1u;
                            for (int i = 0; i < 8; ++i) {
                                const unsigned xG = xBase + 2u * (unsigned)i;
                                scalarSobelStep(p16, p8, /*wide=*/true,
                                                width, height, xG, y,
                                                &partial->sharpSum[py][px]);
                            }
                        }
                    }

                    partial->sumCh[py][px][chanEven] += hsum_u32x4(accEven);
                    partial->sumCh[py][px][chanOdd]  += hsum_u32x4(accOdd);
                    partial->cntCh[py][px][chanEven] += chunks * 8u;
                    partial->cntCh[py][px][chanOdd]  += chunks * 8u;
                    partial->sharpSum[py][px]        += hsum_f32x4(sharpAccV);
                }

                for (; x < x1; ++x) {
                    const unsigned chan = cfa[yParity * 2u + (x & 1u)];
                    const uint32_t v    = wide ? row16[x] : row8[x];

                    partial->sumCh[py][px][chan] += v;
                    partial->cntCh[py][px][chan] += 1u;

                    if (chan == 1u) {
                        uint32_t bin = v >> histShift;
                        if (bin > 127u) bin = 127u;
                        partial->lumaHist[bin] += 1u;
                        scalarSobelStep(p16, p8, wide,
                                        width, height, x, y,
                                        &partial->sharpSum[py][px]);
                    }
                }
            }
        }
    }
}

void NeonStatsEncoder::finalize(const Partial &partial,
                                 uint32_t       pixFmt,
                                 IpaStats      *out) {
    if (!out) return;
    memset(out, 0, sizeof(*out));

    memcpy(out->lumaHist, partial.lumaHist, sizeof(out->lumaHist));

    const float invMax = 1.0f / (float)(is10Bit(pixFmt) ? 1023 : 255);
    for (int py = 0; py < IpaStats::PATCH_Y; ++py) {
        for (int px = 0; px < IpaStats::PATCH_X; ++px) {
            for (int c = 0; c < 3; ++c) {
                const uint32_t n = partial.cntCh[py][px][c];
                if (n == 0u) continue;
                out->rgbMean[py][px][c] =
                    (float)partial.sumCh[py][px][c] / (float)n * invMax;
            }
            out->sharpness[py][px] = partial.sharpSum[py][px];
        }
    }
}

void NeonStatsEncoder::compute(const void *bayer,
                                unsigned    width,
                                unsigned    height,
                                uint32_t    pixFmt,
                                IpaStats   *out) const {
    Partial partial;
    resetPartial(&partial);
    computeRange(bayer, width, height, pixFmt, &partial, 0, IpaStats::PATCH_Y);
    finalize(partial, pixFmt, out);
}

} /* namespace android */
