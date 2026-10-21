#ifndef V4L2_CONTROLS_H
#define V4L2_CONTROLS_H

#include <stdint.h>
#include <linux/videodev2.h>

/* The Tegra camera controls (FRAME_LENGTH, GROUP_HOLD,
 * VI_CAPTURE_TIMEOUT, LINE_TIME_NS) come from the kernel's own uapi
 * header, installed from this device's kernel source
 * (generated_kernel_headers): the HAL keeps no copy of the numbers. */
#include <linux/tegra-v4l2-camera.h>

namespace android {

/* Small batch of V4L2 controls drained by V4l2Device::setControls.
 * Plain data — owner is the call-site (ApplySettingsStage / 3A build
 * it per frame).
 *
 * Mixed-class batches are fine: setControls groups by
 * V4L2_CTRL_ID2CLASS and issues one VIDIOC_S_EXT_CTRLS per class
 * (CAMERA before USER, so frame_length grows before exposure tries
 * to fit). Per-class atomicity is preserved; cross-class ordering
 * is the controlled side-effect. */
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
