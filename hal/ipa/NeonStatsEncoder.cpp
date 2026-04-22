#include "NeonStatsEncoder.h"

#include <stdint.h>
#include <string.h>

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
     * area. */
    uint64_t sumCh[IpaStats::PATCH_Y][IpaStats::PATCH_X][3];
    uint32_t cntCh[IpaStats::PATCH_Y][IpaStats::PATCH_X][3];
    memset(sumCh, 0, sizeof(sumCh));
    memset(cntCh, 0, sizeof(cntCh));

    for (int py = 0; py < IpaStats::PATCH_Y; ++py) {
        const unsigned y0 = by[py];
        const unsigned y1 = by[py + 1];
        for (unsigned y = y0; y < y1; ++y) {
            const uint32_t yParity = y & 1u;
            const uint16_t *row16 = p16 + (size_t)y * width;
            const uint8_t  *row8  = p8  + (size_t)y * width;

            for (int px = 0; px < IpaStats::PATCH_X; ++px) {
                const unsigned x0 = bx[px];
                const unsigned x1 = bx[px + 1];
                for (unsigned x = x0; x < x1; ++x) {
                    const unsigned chan = cfa[yParity * 2u + (x & 1u)];
                    const uint32_t v    = wide ? row16[x] : row8[x];

                    sumCh[py][px][chan] += v;
                    cntCh[py][px][chan] += 1u;

                    if (chan == 1u) {
                        uint32_t bin = v >> histShift;
                        if (bin > 127u) bin = 127u;
                        out->lumaHist[bin] += 1u;
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
        }
    }
}

} /* namespace android */
