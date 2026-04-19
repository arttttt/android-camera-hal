#define LOG_TAG "Cam-ResultMeta"

#include "ResultMetadataBuilder.h"

#include <stdint.h>

#include <system/camera_metadata.h>

#include "sensor/SensorConfig.h"

namespace android {

namespace {

using FrameState = ResultMetadataBuilder::FrameState;

/* Capture timebase — what frame this is and when it was read off the
 * sensor. The framework keys every other per-frame callback off these. */
void writeFrameState(CameraMetadata &cm, const FrameState &fs) {
    int64_t sensorTimestamp = fs.timestampNs;
    int64_t syncFrameNumber = fs.frameNumber;
    cm.update(ANDROID_SENSOR_TIMESTAMP,  &sensorTimestamp, 1);
    cm.update(ANDROID_SYNC_FRAME_NUMBER, &syncFrameNumber, 1);
}

/* AF echo — report whatever the AF controller observed this frame.
 * afMode is echoed here (rather than in the generic-echo section below)
 * because it's conceptually the same "what AF did" group. */
void writeAfReport(CameraMetadata &cm, const FrameState &fs) {
    uint8_t afState      = fs.af.afState;
    uint8_t afMode       = fs.af.afMode;
    float   focusDiopter = fs.af.focusDiopter;
    cm.update(ANDROID_CONTROL_AF_STATE,    &afState,      1);
    cm.update(ANDROID_LENS_FOCUS_DISTANCE, &focusDiopter, 1);
    cm.update(ANDROID_CONTROL_AF_MODE,     &afMode,       1);
}

/* Sensor timing that actually applied on this frame. The framework
 * diffs request vs result to know what landed — absence of a key the
 * app set makes Camera2 throw IllegalArgumentException. */
void writeExposureReport(CameraMetadata &cm, const FrameState &fs,
                          const SensorConfig &cfg) {
    int64_t reportExposureNs    = (int64_t)fs.appliedExposureUs * 1000LL;
    int32_t reportSensitivity   = (fs.appliedGain * 100) / cfg.gainUnit;
    int64_t reportFrameDuration = (int64_t)cfg.frameLenDefault
                                * cfg.lineTimeUs * 1000LL;
    cm.update(ANDROID_SENSOR_EXPOSURE_TIME,  &reportExposureNs,    1);
    cm.update(ANDROID_SENSOR_SENSITIVITY,    &reportSensitivity,   1);
    cm.update(ANDROID_SENSOR_FRAME_DURATION, &reportFrameDuration, 1);
}

/* AE/AWB mode echoes and their derived states.
 *
 * AE: only AE_MODE_OFF is advertised, so there is no loop to converge
 *     or lock — state is always INACTIVE. Revisit when real AE lands.
 *
 * AWB: OFF → INACTIVE; AUTO → LOCKED when the request set AWB_LOCK or
 *      an AF sweep is holding the lock internally, CONVERGED otherwise. */
void writeAeAwbState(CameraMetadata &cm, const FrameState &fs) {
    uint8_t reportAeMode = ANDROID_CONTROL_AE_MODE_OFF;
    if (cm.exists(ANDROID_CONTROL_AE_MODE))
        reportAeMode = *cm.find(ANDROID_CONTROL_AE_MODE).data.u8;
    cm.update(ANDROID_CONTROL_AE_MODE, &reportAeMode, 1);

    uint8_t reportAwbMode = ANDROID_CONTROL_AWB_MODE_OFF;
    if (cm.exists(ANDROID_CONTROL_AWB_MODE))
        reportAwbMode = *cm.find(ANDROID_CONTROL_AWB_MODE).data.u8;
    cm.update(ANDROID_CONTROL_AWB_MODE, &reportAwbMode, 1);

    uint8_t reportAeState = ANDROID_CONTROL_AE_STATE_INACTIVE;
    cm.update(ANDROID_CONTROL_AE_STATE, &reportAeState, 1);

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
}

/* CAPTURE_INTENT echoed from the request; lens aperture / focal length
 * are fixed hardware attributes for this Tegra K1 sensor pair. */
void writeIntentAndLens(CameraMetadata &cm) {
    uint8_t reportIntent = ANDROID_CONTROL_CAPTURE_INTENT_PREVIEW;
    if (cm.exists(ANDROID_CONTROL_CAPTURE_INTENT))
        reportIntent = *cm.find(ANDROID_CONTROL_CAPTURE_INTENT).data.u8;
    cm.update(ANDROID_CONTROL_CAPTURE_INTENT, &reportIntent, 1);

    static const float reportAperture    = 2.0f;
    static const float reportFocalLength = 3.30f;
    cm.update(ANDROID_LENS_APERTURE,     &reportAperture,    1);
    cm.update(ANDROID_LENS_FOCAL_LENGTH, &reportFocalLength, 1);
}

} /* namespace */

void ResultMetadataBuilder::build(CameraMetadata &cm, const FrameState &fs,
                                   const SensorConfig &cfg) {
    writeFrameState     (cm, fs);
    writeAfReport       (cm, fs);
    writeExposureReport (cm, fs, cfg);
    writeAeAwbState     (cm, fs);
    writeIntentAndLens  (cm);
}

}; /* namespace android */
