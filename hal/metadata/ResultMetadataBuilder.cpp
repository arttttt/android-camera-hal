#define LOG_TAG "Cam-ResultMeta"

#include "ResultMetadataBuilder.h"

#include <stdint.h>

#include <system/camera_metadata.h>

#include "sensor/SensorConfig.h"

namespace android {

void ResultMetadataBuilder::build(CameraMetadata &cm, const FrameState &fs,
                                   const SensorConfig &cfg) {
    int64_t sensorTimestamp = fs.timestampNs;
    int64_t syncFrameNumber = fs.frameNumber;

    cm.update(ANDROID_SENSOR_TIMESTAMP,  &sensorTimestamp, 1);
    cm.update(ANDROID_SYNC_FRAME_NUMBER, &syncFrameNumber, 1);

    uint8_t afState = fs.af.afState;
    float   focusDiopter = fs.af.focusDiopter;
    cm.update(ANDROID_CONTROL_AF_STATE,     &afState,      1);
    cm.update(ANDROID_LENS_FOCUS_DISTANCE,  &focusDiopter, 1);

    /* Echo per-frame controls in result metadata. The framework diffs
     * request vs result to know what actually applied; absence of a key
     * the app set makes Camera2 throw IllegalArgumentException. */
    int64_t reportExposureNs  = (int64_t)fs.appliedExposureUs * 1000LL;
    int32_t reportSensitivity = (fs.appliedGain * 100) / cfg.gainUnit;
    int64_t reportFrameDuration = (int64_t)cfg.frameLenDefault
                                * cfg.lineTimeUs * 1000LL;
    cm.update(ANDROID_SENSOR_EXPOSURE_TIME,   &reportExposureNs,    1);
    cm.update(ANDROID_SENSOR_SENSITIVITY,     &reportSensitivity,   1);
    cm.update(ANDROID_SENSOR_FRAME_DURATION,  &reportFrameDuration, 1);

    uint8_t reportAeMode = ANDROID_CONTROL_AE_MODE_OFF;
    if (cm.exists(ANDROID_CONTROL_AE_MODE))
        reportAeMode = *cm.find(ANDROID_CONTROL_AE_MODE).data.u8;
    cm.update(ANDROID_CONTROL_AE_MODE, &reportAeMode, 1);

    uint8_t reportAwbMode = ANDROID_CONTROL_AWB_MODE_OFF;
    if (cm.exists(ANDROID_CONTROL_AWB_MODE))
        reportAwbMode = *cm.find(ANDROID_CONTROL_AWB_MODE).data.u8;
    cm.update(ANDROID_CONTROL_AWB_MODE, &reportAwbMode, 1);

    uint8_t afMode = fs.af.afMode;
    cm.update(ANDROID_CONTROL_AF_MODE, &afMode, 1);

    /* AE state: only AE_MODE_OFF is advertised, so there is no loop to
     * converge or lock — always INACTIVE. Revisit when real AE lands. */
    uint8_t reportAeState = ANDROID_CONTROL_AE_STATE_INACTIVE;
    cm.update(ANDROID_CONTROL_AE_STATE, &reportAeState, 1);

    /* AWB state: OFF → INACTIVE; AUTO → LOCKED when the request set
     * AWB_LOCK or the AF sweep is holding the lock internally,
     * CONVERGED otherwise. */
    uint8_t reportAwbState = ANDROID_CONTROL_AWB_STATE_INACTIVE;
    if (reportAwbMode == ANDROID_CONTROL_AWB_MODE_AUTO) {
        bool awbLocked = (fs.af.afState == ANDROID_CONTROL_AF_STATE_ACTIVE_SCAN);
        if (cm.exists(ANDROID_CONTROL_AWB_LOCK))
            awbLocked = awbLocked || (*cm.find(ANDROID_CONTROL_AWB_LOCK).data.u8
                                      == ANDROID_CONTROL_AWB_LOCK_ON);
        reportAwbState = awbLocked ? ANDROID_CONTROL_AWB_STATE_LOCKED
                                   : ANDROID_CONTROL_AWB_STATE_CONVERGED;
    }
    cm.update(ANDROID_CONTROL_AWB_STATE, &reportAwbState, 1);

    uint8_t reportIntent = ANDROID_CONTROL_CAPTURE_INTENT_PREVIEW;
    if (cm.exists(ANDROID_CONTROL_CAPTURE_INTENT))
        reportIntent = *cm.find(ANDROID_CONTROL_CAPTURE_INTENT).data.u8;
    cm.update(ANDROID_CONTROL_CAPTURE_INTENT, &reportIntent, 1);

    static const float reportAperture    = 2.0f;
    static const float reportFocalLength = 3.30f;
    cm.update(ANDROID_LENS_APERTURE,     &reportAperture,    1);
    cm.update(ANDROID_LENS_FOCAL_LENGTH, &reportFocalLength, 1);
}

}; /* namespace android */
