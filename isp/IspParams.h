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

    void reset();

    /* V4L2 Bayer fourcc → 0..3 matching SRGGB / SGRBG / SGBRG / SBGGR. */
    static uint32_t bayerPhaseFromFourcc(uint32_t fourcc);
};

}; /* namespace android */

#endif /* ISP_PARAMS_H */
