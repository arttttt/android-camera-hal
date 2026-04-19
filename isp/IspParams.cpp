#include "IspParams.h"

#include <string.h>
#include <linux/videodev2.h>

namespace android {

void IspParams::reset() {
    memset(this, 0, sizeof(*this));
    ccm[0] = 1024;
    ccm[4] = 1024;
    ccm[8] = 1024;
}

uint32_t IspParams::bayerPhaseFromFourcc(uint32_t fourcc) {
    switch (fourcc) {
        case V4L2_PIX_FMT_SRGGB10: case V4L2_PIX_FMT_SRGGB8: return 0;
        case V4L2_PIX_FMT_SGRBG10: case V4L2_PIX_FMT_SGRBG8: return 1;
        case V4L2_PIX_FMT_SGBRG10: case V4L2_PIX_FMT_SGBRG8: return 2;
        case V4L2_PIX_FMT_SBGGR10: case V4L2_PIX_FMT_SBGGR8: return 3;
        default: return 0;
    }
}

}; /* namespace android */
