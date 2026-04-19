#ifndef ISP_CALIBRATION_H
#define ISP_CALIBRATION_H

#include <stdint.h>

namespace android {

class IspCalibration {
public:
    /* Stock ISP colour correction matrices from Xiaomi / NVIDIA tuning
     * profiles. Q10 fixed-point (1024 = 1.0x), row-major 3x3 sRGB matrix. */
    static const int16_t *ccmImx179();
    static const int16_t *ccmOv5693();
};

}; /* namespace android */

#endif /* ISP_CALIBRATION_H */
