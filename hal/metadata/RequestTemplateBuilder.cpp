#define LOG_TAG "Cam-ReqTemplate"

#include "RequestTemplateBuilder.h"

#include <stdint.h>

#include <optional>

#include <hardware/camera3.h>
#include <hardware/camera_common.h>
#include "CameraMetadataCompat.h"
#include <utils/misc.h>


namespace android {

namespace {

/* Request envelope — identifier, lens focus, crop region. The crop
 * region defaults to the full active array (no zoom). */
void writeFrameState(CameraMetadata &cm, const mocha::camera::Capability &cap) {
    static const int32_t requestId = 0;
    cm.update(ANDROID_REQUEST_ID, &requestId, 1);

    static const float lensFocusDistance = 0.0f;
    cm.update(ANDROID_LENS_FOCUS_DISTANCE, &lensFocusDistance, 1);

    /* The whole active array, in its own coordinates. */
    const mocha::camera::Rect active = cap.sensor.activeArray;
    const int32_t scalerCropRegion[] = {
        0,                          0,
        active.width,               active.height
    };
    cm.update(ANDROID_SCALER_CROP_REGION, scalerCropRegion, NELEM(scalerCropRegion));
}

/* JPEG encoder defaults — thumbnail geometry, orientation. All
 * overwritten at capture time by the app. GPS is not offered: the HAL
 * writes no GPS into the JPEG, and apps that geotag write it
 * themselves. */
void writeJpegDefaults(CameraMetadata &cm) {
    static const int32_t jpegThumbnailSize[] = {
        0, 0
    };
    cm.update(ANDROID_JPEG_THUMBNAIL_SIZE, jpegThumbnailSize, NELEM(jpegThumbnailSize));

    static const uint8_t jpegThumbnailQuality = 50;
    cm.update(ANDROID_JPEG_THUMBNAIL_QUALITY, &jpegThumbnailQuality, 1);

    static const int32_t jpegOrientation = 0;
    cm.update(ANDROID_JPEG_ORIENTATION, &jpegOrientation, 1);
}

/* Statistics modes — face detection OFF. Histogram and sharpness-map
 * modes are left out: metadata_definitions.xml tags them FUTURE ("do
 * not implement or use it") and keeps them out of the public API. */
void writeStatisticsDefaults(CameraMetadata &cm) {
    static const uint8_t statisticsFaceDetectMode = ANDROID_STATISTICS_FACE_DETECT_MODE_OFF;
    cm.update(ANDROID_STATISTICS_FACE_DETECT_MODE, &statisticsFaceDetectMode, 1);
}

/* The AE target fps range a template starts with, from those the
 * camera advertises, at the rate its largest output runs: video a
 * fixed one, so a recording keeps its rate in the dark; the rest the
 * widest under it, so auto exposure may lengthen the frame before it
 * adds gain (the emulated camera's [15, 30] and [30, 30]). */
mocha::camera::FpsRange templateFpsRange(const mocha::camera::Capability &cap, bool video) {
    int64_t fastestNs = 0;
    uint64_t largest = 0;
    for (const mocha::camera::Capability::StreamConfig &st : cap.streams) {
        const uint64_t area = (uint64_t)st.size.width * st.size.height;
        if (area > largest && st.minFrameDurationNs > 0) {
            largest   = area;
            fastestNs = st.minFrameDurationNs;
        }
    }
    const int32_t rate = fastestNs > 0 ? (int32_t)((1000000000LL + fastestNs / 2) / fastestNs) : 0;

    std::optional<mocha::camera::FpsRange> pick;
    for (const mocha::camera::FpsRange &r : cap.ae.targetFpsRanges) {
        if (r.max != rate) continue;
        if (video ? r.min == r.max : (!pick || r.min < pick->min))
            pick = r;
        if (video && pick) break;
    }
    if (pick) return *pick;
    /* The probe always advertises some range. */
    return cap.ae.targetFpsRanges.empty() ? mocha::camera::FpsRange{}
                                          : cap.ae.targetFpsRanges.front();
}

/* 3A + capture-intent defaults. capture_intent varies by template
 * type; AF mode depends on camera facing (the front lens has no VCM
 * so only MODE_OFF is valid). */
void writeControlDefaults(CameraMetadata &cm, int type,
                          const mocha::camera::Capability &cap) {
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

    /* 3A on by default (AUTO / AE ON / AWB AUTO), as camera3.h asks of
     * every template but MANUAL, which hands 3A to the app: an app
     * that takes a template as is gets a preview that adapts to the
     * scene. */
    const bool manual = (type == CAMERA3_TEMPLATE_MANUAL);
    const uint8_t controlMode = manual ? ANDROID_CONTROL_MODE_OFF
                                       : ANDROID_CONTROL_MODE_AUTO;
    cm.update(ANDROID_CONTROL_MODE, &controlMode, 1);

    static const uint8_t controlEffectMode = ANDROID_CONTROL_EFFECT_MODE_OFF;
    cm.update(ANDROID_CONTROL_EFFECT_MODE, &controlEffectMode, 1);

    /* The only scene mode the static metadata advertises. */
    static const uint8_t controlSceneMode = ANDROID_CONTROL_SCENE_MODE_DISABLED;
    cm.update(ANDROID_CONTROL_SCENE_MODE, &controlSceneMode, 1);

    const uint8_t controlAeMode = manual ? ANDROID_CONTROL_AE_MODE_OFF
                                         : ANDROID_CONTROL_AE_MODE_ON;
    cm.update(ANDROID_CONTROL_AE_MODE, &controlAeMode, 1);

    static const uint8_t controlAeLock = ANDROID_CONTROL_AE_LOCK_OFF;
    cm.update(ANDROID_CONTROL_AE_LOCK, &controlAeLock, 1);

    /* The whole active array: no particular region. AWB takes none
     * (MAX_REGIONS), so it has no key. */
    const mocha::camera::Rect active = cap.sensor.activeArray;
    const int32_t wholeRegion[] = {
        0,                          0,
        active.width,               active.height,
        1000
    };
    cm.update(ANDROID_CONTROL_AE_REGIONS, wholeRegion, NELEM(wholeRegion));
    cm.update(ANDROID_CONTROL_AF_REGIONS, wholeRegion, NELEM(wholeRegion));

    static const int32_t controlAeExposureCompensation = 0;
    cm.update(ANDROID_CONTROL_AE_EXPOSURE_COMPENSATION, &controlAeExposureCompensation, 1);

    const bool video = type == CAMERA3_TEMPLATE_VIDEO_RECORD
                    || type == CAMERA3_TEMPLATE_VIDEO_SNAPSHOT;
    const mocha::camera::FpsRange fps = templateFpsRange(cap, video);
    const int32_t controlAeTargetFpsRange[] = { fps.min, fps.max };
    cm.update(ANDROID_CONTROL_AE_TARGET_FPS_RANGE, controlAeTargetFpsRange, NELEM(controlAeTargetFpsRange));

    /* The camera's mains; the manual template's exposure is the app's. */
    const uint8_t controlAeAntibandingMode = manual
        ? ANDROID_CONTROL_AE_ANTIBANDING_MODE_OFF
        : (uint8_t)cap.ae.antibandingDefault;
    cm.update(ANDROID_CONTROL_AE_ANTIBANDING_MODE, &controlAeAntibandingMode, 1);

    const uint8_t controlAwbMode = manual ? ANDROID_CONTROL_AWB_MODE_OFF
                                          : ANDROID_CONTROL_AWB_MODE_AUTO;
    cm.update(ANDROID_CONTROL_AWB_MODE, &controlAwbMode, 1);

    static const uint8_t controlAwbLock = ANDROID_CONTROL_AWB_LOCK_OFF;
    cm.update(ANDROID_CONTROL_AWB_LOCK, &controlAwbLock, 1);

    /* No flash unit on mocha, but android.flash.mode is required in
     * every request/result regardless (hwlevel=legacy). Seeding the
     * templates is enough: the request→result echo carries it into
     * every capture result. Camera2's ExifUtil unboxes it without a
     * null check. */
    static const uint8_t flashMode = ANDROID_FLASH_MODE_OFF;
    cm.update(ANDROID_FLASH_MODE, &flashMode, 1);

    /* AF Auto where the lens moves; a fixed lens offers Off only. */
    uint8_t controlAfMode = ANDROID_CONTROL_AF_MODE_OFF;
    for (mocha::camera::AfMode m : cap.af.modes)
        if (m == mocha::camera::AfMode::Auto) controlAfMode = ANDROID_CONTROL_AF_MODE_AUTO;
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

camera_metadata_t *RequestTemplateBuilder::build(int type,
                                                  const mocha::camera::Capability &cap) {
    CameraMetadata cm;
    writeFrameState         (cm, cap);
    writeJpegDefaults       (cm);
    writeStatisticsDefaults (cm);
    writeControlDefaults    (cm, type, cap);
    return cm.release();
}

}; /* namespace android */
