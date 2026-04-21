#ifndef V4L2_CONTROLS_H
#define V4L2_CONTROLS_H

#include <stdint.h>

namespace android {

/* Small batch of V4L2 controls for a single VIDIOC_S_EXT_CTRLS call.
 * Plain data — owner is the call-site (ApplySettingsStage builds it
 * per frame, V4l2Device::setControls drains it into the driver).
 *
 * The batch is expected to carry controls from a single V4L2 control
 * class per call (driver rejects mixed-class batches unless
 * ctrl_class == 0, which older kernels don't accept). Today every
 * consumer uses V4L2_CTRL_CLASS_USER (exposure, gain). */
struct V4l2Controls {
    static const int MAX_ENTRIES = 8;

    uint32_t ids   [MAX_ENTRIES];
    int32_t  values[MAX_ENTRIES];
    int      count;

    V4l2Controls() : count(0) {}

    bool add(uint32_t id, int32_t value) {
        if (count >= MAX_ENTRIES) return false;
        ids   [count] = id;
        values[count] = value;
        ++count;
        return true;
    }
};

} /* namespace android */

#endif /* V4L2_CONTROLS_H */
