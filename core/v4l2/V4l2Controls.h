#ifndef V4L2_CONTROLS_H
#define V4L2_CONTROLS_H

#include <stdint.h>
#include <linux/videodev2.h>

/* Tegra camera vendor CID — the kernel sensor drivers expose
 * frame_length (= sensor lines per frame, dictating both FPS and the
 * max exposure that fits in one frame) as 0x009a2000. Not in mainline
 * V4L2 headers; defined here so every consumer sharing this batch
 * struct can write to it without re-declaring. */
#ifndef V4L2_CID_FRAME_LENGTH
#define V4L2_CID_FRAME_LENGTH   (V4L2_CTRL_CLASS_CAMERA | 0x2000)
#endif

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
