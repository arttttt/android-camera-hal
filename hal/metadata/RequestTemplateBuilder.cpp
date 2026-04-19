#define LOG_TAG "Cam-ReqTemplate"

#include "RequestTemplateBuilder.h"

#include <stdint.h>

#include <hardware/camera3.h>
#include <hardware/camera_common.h>
#include <camera/CameraMetadata.h>
#include <utils/misc.h>

#include "V4l2Device.h"

namespace android {

namespace {

/* Request envelope — identifier, lens focus, crop region. The crop
 * region defaults to the full active array (no zoom). */
void writeFrameState(CameraMetadata &cm, V4l2Device *dev) {
    static const int32_t requestId = 0;
    cm.update(ANDROID_REQUEST_ID, &requestId, 1);

    static const float lensFocusDistance = 0.0f;
    cm.update(ANDROID_LENS_FOCUS_DISTANCE, &lensFocusDistance, 1);

    auto sensorSize = dev->sensorResolution();
    const int32_t scalerCropRegion[] = {
        0,                          0,
        (int32_t)sensorSize.width,  (int32_t)sensorSize.height
    };
    cm.update(ANDROID_SCALER_CROP_REGION, scalerCropRegion, NELEM(scalerCropRegion));
}

/* JPEG encoder defaults — thumbnail geometry, GPS placeholders,
 * orientation. All overwritten at capture time by the app. */
void writeJpegDefaults(CameraMetadata &cm) {
    static const int32_t jpegThumbnailSize[] = {
        0, 0
    };
    cm.update(ANDROID_JPEG_THUMBNAIL_SIZE, jpegThumbnailSize, NELEM(jpegThumbnailSize));

    static const uint8_t jpegThumbnailQuality = 50;
    cm.update(ANDROID_JPEG_THUMBNAIL_QUALITY, &jpegThumbnailQuality, 1);

    static const double jpegGpsCoordinates[] = {
        0, 0
    };
    cm.update(ANDROID_JPEG_GPS_COORDINATES, jpegGpsCoordinates, NELEM(jpegGpsCoordinates));

    static const uint8_t jpegGpsProcessingMethod[32] = "None";
    cm.update(ANDROID_JPEG_GPS_PROCESSING_METHOD, jpegGpsProcessingMethod, NELEM(jpegGpsProcessingMethod));

    static const int64_t jpegGpsTimestamp = 0;
    cm.update(ANDROID_JPEG_GPS_TIMESTAMP, &jpegGpsTimestamp, 1);

    static const int32_t jpegOrientation = 0;
    cm.update(ANDROID_JPEG_ORIENTATION, &jpegOrientation, 1);
}

/* Statistics modes — all OFF by default; no histogram/sharpness-map
 * generation happens unless an app opts in. */
void writeStatisticsDefaults(CameraMetadata &cm) {
    static const uint8_t statisticsFaceDetectMode = ANDROID_STATISTICS_FACE_DETECT_MODE_OFF;
    cm.update(ANDROID_STATISTICS_FACE_DETECT_MODE, &statisticsFaceDetectMode, 1);

    static const uint8_t statisticsHistogramMode = ANDROID_STATISTICS_HISTOGRAM_MODE_OFF;
    cm.update(ANDROID_STATISTICS_HISTOGRAM_MODE, &statisticsHistogramMode, 1);

    static const uint8_t statisticsSharpnessMapMode = ANDROID_STATISTICS_SHARPNESS_MAP_MODE_OFF;
    cm.update(ANDROID_STATISTICS_SHARPNESS_MAP_MODE, &statisticsSharpnessMapMode, 1);
}

/* 3A + capture-intent defaults. capture_intent varies by template
 * type; AF mode depends on camera facing (the front lens has no VCM
 * so only MODE_OFF is valid). */
void writeControlDefaults(CameraMetadata &cm, int type, V4l2Device *dev, int facing) {
    uint8_t controlCaptureIntent = 0;
    switch (type) {
        case CAMERA3_TEMPLATE_PREVIEW:          controlCaptureIntent = ANDROID_CONTROL_CAPTURE_INTENT_PREVIEW;             break;
        case CAMERA3_TEMPLATE_STILL_CAPTURE:    controlCaptureIntent = ANDROID_CONTROL_CAPTURE_INTENT_STILL_CAPTURE;       break;
        case CAMERA3_TEMPLATE_VIDEO_RECORD:     controlCaptureIntent = ANDROID_CONTROL_CAPTURE_INTENT_VIDEO_RECORD;        break;
        case CAMERA3_TEMPLATE_VIDEO_SNAPSHOT:   controlCaptureIntent = ANDROID_CONTROL_CAPTURE_INTENT_VIDEO_SNAPSHOT;      break;
        case CAMERA3_TEMPLATE_ZERO_SHUTTER_LAG: controlCaptureIntent = ANDROID_CONTROL_CAPTURE_INTENT_ZERO_SHUTTER_LAG;    break;
        default:                                controlCaptureIntent = ANDROID_CONTROL_CAPTURE_INTENT_CUSTOM;              break;
    }
    cm.update(ANDROID_CONTROL_CAPTURE_INTENT, &controlCaptureIntent, 1);

    static const uint8_t controlMode = ANDROID_CONTROL_MODE_OFF;
    cm.update(ANDROID_CONTROL_MODE, &controlMode, 1);

    static const uint8_t controlEffectMode = ANDROID_CONTROL_EFFECT_MODE_OFF;
    cm.update(ANDROID_CONTROL_EFFECT_MODE, &controlEffectMode, 1);

    static const uint8_t controlSceneMode = ANDROID_CONTROL_SCENE_MODE_FACE_PRIORITY;
    cm.update(ANDROID_CONTROL_SCENE_MODE, &controlSceneMode, 1);

    static const uint8_t controlAeMode = ANDROID_CONTROL_AE_MODE_OFF;
    cm.update(ANDROID_CONTROL_AE_MODE, &controlAeMode, 1);

    static const uint8_t controlAeLock = ANDROID_CONTROL_AE_LOCK_OFF;
    cm.update(ANDROID_CONTROL_AE_LOCK, &controlAeLock, 1);

    auto sensorSize = dev->sensorResolution();
    const int32_t controlAeRegions[] = {
        0,                          0,
        (int32_t)sensorSize.width,  (int32_t)sensorSize.height,
        1000
    };
    cm.update(ANDROID_CONTROL_AE_REGIONS,  controlAeRegions, NELEM(controlAeRegions));
    cm.update(ANDROID_CONTROL_AWB_REGIONS, controlAeRegions, NELEM(controlAeRegions));
    cm.update(ANDROID_CONTROL_AF_REGIONS,  controlAeRegions, NELEM(controlAeRegions));

    static const int32_t controlAeExposureCompensation = 0;
    cm.update(ANDROID_CONTROL_AE_EXPOSURE_COMPENSATION, &controlAeExposureCompensation, 1);

    static const int32_t controlAeTargetFpsRange[] = {
        15, 30
    };
    cm.update(ANDROID_CONTROL_AE_TARGET_FPS_RANGE, controlAeTargetFpsRange, NELEM(controlAeTargetFpsRange));

    static const uint8_t controlAeAntibandingMode = ANDROID_CONTROL_AE_ANTIBANDING_MODE_OFF;
    cm.update(ANDROID_CONTROL_AE_ANTIBANDING_MODE, &controlAeAntibandingMode, 1);

    static const uint8_t controlAwbMode = ANDROID_CONTROL_AWB_MODE_OFF;
    cm.update(ANDROID_CONTROL_AWB_MODE, &controlAwbMode, 1);

    static const uint8_t controlAwbLock = ANDROID_CONTROL_AWB_LOCK_OFF;
    cm.update(ANDROID_CONTROL_AWB_LOCK, &controlAwbLock, 1);

    uint8_t controlAfMode = (facing == CAMERA_FACING_BACK) ?
        ANDROID_CONTROL_AF_MODE_AUTO : ANDROID_CONTROL_AF_MODE_OFF;
    cm.update(ANDROID_CONTROL_AF_MODE, &controlAfMode, 1);

    static const uint8_t controlAeState = ANDROID_CONTROL_AE_STATE_CONVERGED;
    cm.update(ANDROID_CONTROL_AE_STATE, &controlAeState, 1);
    static const uint8_t controlAfState = ANDROID_CONTROL_AF_STATE_INACTIVE;
    cm.update(ANDROID_CONTROL_AF_STATE, &controlAfState, 1);
    static const uint8_t controlAwbState = ANDROID_CONTROL_AWB_STATE_INACTIVE;
    cm.update(ANDROID_CONTROL_AWB_STATE, &controlAwbState, 1);

    static const uint8_t controlVideoStabilizationMode = ANDROID_CONTROL_VIDEO_STABILIZATION_MODE_OFF;
    cm.update(ANDROID_CONTROL_VIDEO_STABILIZATION_MODE, &controlVideoStabilizationMode, 1);

    static const int32_t controlAePrecaptureId = ANDROID_CONTROL_AE_PRECAPTURE_TRIGGER_IDLE;
    cm.update(ANDROID_CONTROL_AE_PRECAPTURE_ID, &controlAePrecaptureId, 1);

    static const int32_t controlAfTriggerId = 0;
    cm.update(ANDROID_CONTROL_AF_TRIGGER_ID, &controlAfTriggerId, 1);
}

} /* namespace */

camera_metadata_t *RequestTemplateBuilder::build(int type, V4l2Device *dev, int facing) {
    CameraMetadata cm;
    writeFrameState         (cm, dev);
    writeJpegDefaults       (cm);
    writeStatisticsDefaults (cm);
    writeControlDefaults    (cm, type, dev, facing);
    return cm.release();
}

}; /* namespace android */
