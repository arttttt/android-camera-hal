#include "IspCalibration.h"

namespace android {

const int16_t *IspCalibration::ccmImx179() {
    /* imx179_primax_v2.27.isp, srgbMatrix (D50)
     *   R' =  1.884*R - 0.824*G - 0.059*B
     *   G' = -0.262*R + 1.615*G - 0.354*B
     *   B' =  0.051*R - 0.803*G + 1.751*B
     */
    static const int16_t ccm[9] = {
         1930,  -844,   -61,
         -268,  1654,  -362,
           52,  -822,  1793,
    };
    return ccm;
}

const int16_t *IspCalibration::ccmOv5693() {
    /* ov5693_sunny_v2.13.isp
     *   R' =  1.867*R - 0.841*G - 0.026*B
     *   G' = -0.262*R + 1.541*G - 0.279*B
     *   B' = -0.015*R - 0.647*G + 1.662*B
     */
    static const int16_t ccm[9] = {
         1912,  -861,   -27,
         -268,  1578,  -286,
          -15,  -663,  1702,
    };
    return ccm;
}

}; /* namespace android */
