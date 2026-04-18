#ifndef ISP_CALIBRATION_H
#define ISP_CALIBRATION_H

#include <stdint.h>

/* Stock ISP color correction matrices from Xiaomi/NVIDIA calibration profiles.
 * Q10 fixed-point (1024 = 1.0x). Row-major: {R_from_R, R_from_G, R_from_B, ...} */

static const int16_t ccm_imx179[9] = {  /* imx179_primax_v2.27.isp, srgbMatrix (D50) */
     1930,  -844,   -61,   /* R' =  1.884*R - 0.824*G - 0.059*B */
     -268,  1654,  -362,   /* G' = -0.262*R + 1.615*G - 0.354*B */
       52,  -822,  1793,   /* B' =  0.051*R - 0.803*G + 1.751*B */
};

static const int16_t ccm_ov5693[9] = {  /* ov5693_sunny_v2.13.isp */
     1912,  -861,   -27,   /* R' =  1.867*R - 0.841*G - 0.026*B */
     -268,  1578,  -286,   /* G' = -0.262*R + 1.541*G - 0.279*B */
      -15,  -663,  1702,   /* B' = -0.015*R - 0.647*G + 1.662*B */
};

#endif // ISP_CALIBRATION_H
