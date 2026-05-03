#define LOG_TAG "Cam-ResultMeta"

#include "ResultMetadataBuilder.h"

#include <stdint.h>

#include <system/camera_metadata.h>

#include "sensor/SensorConfig.h"

namespace android {

using FrameState = ResultMetadataBuilder::FrameState;

void ResultMetadataBuilder::buildBaseMetadata(CameraMetadata &cm,
                                               const FrameState &fs) {
    /* Capture timebase + frame identity. */
    int64_t sensorTimestamp = fs.timestampNs;
    int64_t syncFrameNumber = fs.frameNumber;
    cm.update(ANDROID_SENSOR_TIMESTAMP,  &sensorTimestamp, 1);
    cm.update(ANDROID_SYNC_FRAME_NUMBER, &syncFrameNumber, 1);

    /* Capture intent echoed; lens aperture / focal length are fixed
     * hardware attributes for this Tegra K1 sensor pair. */
    uint8_t reportIntent = ANDROID_CONTROL_CAPTURE_INTENT_PREVIEW;
    if (cm.exists(ANDROID_CONTROL_CAPTURE_INTENT))
        reportIntent = *cm.find(ANDROID_CONTROL_CAPTURE_INTENT).data.u8;
    cm.update(ANDROID_CONTROL_CAPTURE_INTENT, &reportIntent, 1);

    static const float reportAperture    = 2.0f;
    static const float reportFocalLength = 3.30f;
    cm.update(ANDROID_LENS_APERTURE,     &reportAperture,    1);
    cm.update(ANDROID_LENS_FOCAL_LENGTH, &reportFocalLength, 1);
}

void ResultMetadataBuilder::buildAfMetadata(CameraMetadata &cm,
                                             const FrameState &fs) {
    uint8_t afState      = fs.af.afState;
    uint8_t afMode       = fs.af.afMode;
    float   focusDiopter = fs.af.focusDiopter;
    cm.update(ANDROID_CONTROL_AF_STATE,    &afState,      1);
    cm.update(ANDROID_LENS_FOCUS_DISTANCE, &focusDiopter, 1);
    cm.update(ANDROID_CONTROL_AF_MODE,     &afMode,       1);
}

void ResultMetadataBuilder::buildAeMetadata(CameraMetadata &cm,
                                             const FrameState &fs,
                                             const SensorConfig &cfg) {
    /* Sensor timing applied on this frame. The framework diffs
     * request vs result to know what landed. */
    int64_t reportExposureNs    = (int64_t)fs.appliedExposureUs * 1000LL;
    int32_t reportSensitivity   = cfg.gainToIso(fs.appliedGain);
    int64_t reportFrameDuration = (int64_t)cfg.frameLenDefault
                                * cfg.lineTimeUs * 1000LL;
    cm.update(ANDROID_SENSOR_EXPOSURE_TIME,  &reportExposureNs,    1);
    cm.update(ANDROID_SENSOR_SENSITIVITY,    &reportSensitivity,   1);
    cm.update(ANDROID_SENSOR_FRAME_DURATION, &reportFrameDuration, 1);

    /* AE mode echo + state derivation. AE: OFF → INACTIVE; LOCK_ON
     * → LOCKED; aeConverged → CONVERGED; otherwise SEARCHING. */
    uint8_t reportAeMode = ANDROID_CONTROL_AE_MODE_OFF;
    if (cm.exists(ANDROID_CONTROL_AE_MODE))
        reportAeMode = *cm.find(ANDROID_CONTROL_AE_MODE).data.u8;
    cm.update(ANDROID_CONTROL_AE_MODE, &reportAeMode, 1);

    uint8_t aeLock = ANDROID_CONTROL_AE_LOCK_OFF;
    if (cm.exists(ANDROID_CONTROL_AE_LOCK))
        aeLock = *cm.find(ANDROID_CONTROL_AE_LOCK).data.u8;

    uint8_t reportAeState = ANDROID_CONTROL_AE_STATE_INACTIVE;
    if (reportAeMode != ANDROID_CONTROL_AE_MODE_OFF) {
        if (aeLock == ANDROID_CONTROL_AE_LOCK_ON) {
            reportAeState = ANDROID_CONTROL_AE_STATE_LOCKED;
        } else if (fs.aeConverged) {
            reportAeState = ANDROID_CONTROL_AE_STATE_CONVERGED;
        } else {
            reportAeState = ANDROID_CONTROL_AE_STATE_SEARCHING;
        }
    }
    cm.update(ANDROID_CONTROL_AE_STATE, &reportAeState, 1);

    /* Throttled diag — every 32 frames. */
    if ((fs.frameNumber & 0x1f) == 0u) {
        ALOGD("AE: frame=%u aeMode=%u aeLock=%u aeConverged=%d aeState=%u",
              fs.frameNumber, reportAeMode, aeLock,
              fs.aeConverged ? 1 : 0, reportAeState);
    }
}

void ResultMetadataBuilder::buildAwbMetadata(CameraMetadata &cm,
                                              const FrameState &fs) {
    uint8_t reportAwbMode = ANDROID_CONTROL_AWB_MODE_OFF;
    if (cm.exists(ANDROID_CONTROL_AWB_MODE))
        reportAwbMode = *cm.find(ANDROID_CONTROL_AWB_MODE).data.u8;
    cm.update(ANDROID_CONTROL_AWB_MODE, &reportAwbMode, 1);

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

    if ((fs.frameNumber & 0x1f) == 0u) {
        ALOGD("AWB: frame=%u awbMode=%u awbState=%u",
              fs.frameNumber, reportAwbMode, reportAwbState);
    }
}

void ResultMetadataBuilder::build(CameraMetadata &cm, const FrameState &fs,
                                   const SensorConfig &cfg) {
    buildBaseMetadata(cm, fs);
    buildAfMetadata  (cm, fs);
    buildAeMetadata  (cm, fs, cfg);
    buildAwbMetadata (cm, fs);
}

}; /* namespace android */
