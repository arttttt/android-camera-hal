#ifndef V4L2_CONTROLS_H
#define V4L2_CONTROLS_H

#include <stdint.h>
#include <linux/videodev2.h>

/* Tegra camera vendor CIDs (kernel include/media/camera_common.h), not
 * in the mainline V4L2 headers; defined here once for the whole HAL.
 *  - FRAME_LENGTH: sensor lines per frame -- the frame period, and the
 *    longest exposure that fits in it. Its range is the current mode's
 *    (minimum = the mode's own frame length). The HAL decides it; the
 *    drivers no longer stretch it to fit an exposure.
 *  - GROUP_HOLD: latch the writes between 1 and 0 on the same frame.
 *  - VI_CAPTURE_TIMEOUT: on the capture node -- how long VI waits for a
 *    frame, µs, set from the period of the frames in flight.
 *  - LINE_TIME_NS: read-only, the current mode's line time. */
#define V4L2_CID_TEGRA_CAMERA_BASE  (V4L2_CTRL_CLASS_CAMERA | 0x2000)
#define V4L2_CID_FRAME_LENGTH       (V4L2_CID_TEGRA_CAMERA_BASE + 0)
#define V4L2_CID_GROUP_HOLD         (V4L2_CID_TEGRA_CAMERA_BASE + 3)
#define V4L2_CID_VI_CAPTURE_TIMEOUT (V4L2_CID_TEGRA_CAMERA_BASE + 10)
#define V4L2_CID_LINE_TIME_NS       (V4L2_CID_TEGRA_CAMERA_BASE + 11)

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
