#ifndef HAL_IPA_IPA_FRAME_META_H
#define HAL_IPA_IPA_FRAME_META_H

#include <stdint.h>

#include <system/camera_metadata.h>

namespace android {

/* Per-frame framework control state the IPA needs to decide what to
 * update. Extracted from ctx.request.settings in StatsProcessStage so
 * the IPA stays ignorant of CameraMetadata parsing.
 *
 * Each field defaults to the camera3 spec's "auto" value when the
 * request does not set the corresponding key, so an IPA can treat a
 * missing key as permission to run. */
struct IpaFrameMeta {
    /* ANDROID_CONTROL_AE_MODE — OFF means the framework controls
     * exposure / gain directly; the IPA must not push either. */
    uint8_t aeMode;

    /* ANDROID_CONTROL_AE_LOCK — ON means the IPA must hold the last
     * exposure / gain decision. */
    uint8_t aeLock;

    /* ANDROID_CONTROL_AWB_MODE — OFF means the framework controls
     * colour-correction gains directly; presets (INCANDESCENT etc.)
     * eventually map to pre-tuned WB priors but today every non-AUTO
     * value freezes the gains at their last auto decision. */
    uint8_t awbMode;

    /* ANDROID_CONTROL_AWB_LOCK — ON means the IPA must hold the last
     * WB gains. */
    uint8_t awbLock;

    IpaFrameMeta()
        : aeMode(ANDROID_CONTROL_AE_MODE_ON),
          aeLock(ANDROID_CONTROL_AE_LOCK_OFF),
          awbMode(ANDROID_CONTROL_AWB_MODE_AUTO),
          awbLock(ANDROID_CONTROL_AWB_LOCK_OFF) {}
};

} /* namespace android */

#endif /* HAL_IPA_IPA_FRAME_META_H */
