#ifndef ISP_PARAMS_H
#define ISP_PARAMS_H

#include <stdint.h>

namespace android {

/* Per-frame ISP parameter block. Layout matches the std430 SSBO
 * the Vulkan demosaic + WB + CCM shader expects — edit the two in
 * lock-step or the shader reads garbage. Plain-data on purpose so
 * memcpy into the mapped param buffer is well-defined. */
struct IspParams {
    uint32_t width;
    uint32_t height;
    uint32_t bayerPhase;
    uint32_t is16bit;
    uint32_t wbR;
    uint32_t wbG;
    uint32_t wbB;
    uint32_t doIsp;
    int32_t  ccm[9];
    /* Optical-black bias in the sensor's native range (10-bit for
     * SRGGB10 / SGRBG10 / SGBRG10 / SBGGR10, 8-bit for the SBGGR8
     * family). Subtracted and rescaled in the demosaic shader before
     * any colour processing. 0 disables the correction. */
    uint32_t blackLevel;

    /* Precomputed 255 / (maxRaw - blackLevel) so the shader does one
     * multiply per raw sample instead of the emulated integer divide
     * Kepler pays for. Populated by the host in fillParams — keep in
     * lock-step with blackLevel and is16bit. */
    float denomRecip255;

    /* 256-entry sRGB encode LUT: uint8 values stored one-per-uint.
     * Replaces the per-channel pow(x, 1/2.4) in the shader when
     * doIsp is set — 3 pow() calls per pixel (Kepler SFU, ~8-12
     * cycles each) become 3 shared-memory reads per pixel. Filled
     * once by the host in fillParams from a cached table. */
    uint32_t gammaLut[256];

    void reset();

    /* V4L2 Bayer fourcc → 0..3 matching SRGGB / SGRBG / SGBRG / SBGGR. */
    static uint32_t bayerPhaseFromFourcc(uint32_t fourcc);
};

}; /* namespace android */

#endif /* ISP_PARAMS_H */
