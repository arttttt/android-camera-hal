#include "ResultMetadataBuilder.h"

#include <stdint.h>

#include <system/camera_metadata.h>

namespace android {

using mocha::camera::Core3AResult;

void ResultMetadataBuilder::buildBase(CameraMetadata &cm, int64_t timestampNs,
                                       uint32_t frameNumber, float focalLengthMm) {
    /* Capture timebase + frame identity. */
    int64_t sensorTimestamp = timestampNs;
    int64_t syncFrameNumber = frameNumber;
    cm.update(ANDROID_SENSOR_TIMESTAMP,  &sensorTimestamp, 1);
    cm.update(ANDROID_SYNC_FRAME_NUMBER, &syncFrameNumber, 1);

    /* Capture intent echoed; lens aperture / focal length are fixed
     * hardware attributes. The focal length is the module's, the same
     * value LENS_INFO_AVAILABLE_FOCAL_LENGTHS advertises. */
    uint8_t reportIntent = ANDROID_CONTROL_CAPTURE_INTENT_PREVIEW;
    if (cm.exists(ANDROID_CONTROL_CAPTURE_INTENT))
        reportIntent = *cm.find(ANDROID_CONTROL_CAPTURE_INTENT).data.u8;
    cm.update(ANDROID_CONTROL_CAPTURE_INTENT, &reportIntent, 1);

    static const float reportAperture = 2.0f;
    cm.update(ANDROID_LENS_APERTURE,     &reportAperture, 1);
    cm.update(ANDROID_LENS_FOCAL_LENGTH, &focalLengthMm,  1);

    /* Dynamic-only key — never present in the request echo, so it
     * must be written here. No flash unit → always UNAVAILABLE. */
    static const uint8_t flashState = ANDROID_FLASH_STATE_UNAVAILABLE;
    cm.update(ANDROID_FLASH_STATE, &flashState, 1);
}

void ResultMetadataBuilder::buildAf(CameraMetadata &cm, const Core3AResult &r) {
    uint8_t afState      = (uint8_t)r.afState;
    uint8_t afMode       = (uint8_t)r.afMode;
    float   focusDiopter = r.focusDistanceDiopters;
    cm.update(ANDROID_CONTROL_AF_STATE,    &afState,      1);
    cm.update(ANDROID_LENS_FOCUS_DISTANCE, &focusDiopter, 1);
    cm.update(ANDROID_CONTROL_AF_MODE,     &afMode,       1);
}

void ResultMetadataBuilder::buildAe(CameraMetadata &cm, const Core3AResult &r) {
    /* Sensor timing applied on this frame. The framework diffs
     * request vs result to know what landed. */
    int64_t exposureNs    = r.exposureTimeNs;
    int32_t sensitivity   = r.sensitivityIso;
    int64_t frameDuration = r.frameDurationNs;
    cm.update(ANDROID_SENSOR_EXPOSURE_TIME,  &exposureNs,    1);
    cm.update(ANDROID_SENSOR_SENSITIVITY,    &sensitivity,   1);
    cm.update(ANDROID_SENSOR_FRAME_DURATION, &frameDuration, 1);

    uint8_t aeMode = ANDROID_CONTROL_AE_MODE_OFF;
    if (cm.exists(ANDROID_CONTROL_AE_MODE))
        aeMode = *cm.find(ANDROID_CONTROL_AE_MODE).data.u8;
    cm.update(ANDROID_CONTROL_AE_MODE, &aeMode, 1);

    uint8_t aeState = (uint8_t)r.aeState;
    cm.update(ANDROID_CONTROL_AE_STATE, &aeState, 1);
}

void ResultMetadataBuilder::buildAwb(CameraMetadata &cm, const Core3AResult &r) {
    uint8_t awbMode = ANDROID_CONTROL_AWB_MODE_OFF;
    if (cm.exists(ANDROID_CONTROL_AWB_MODE))
        awbMode = *cm.find(ANDROID_CONTROL_AWB_MODE).data.u8;
    cm.update(ANDROID_CONTROL_AWB_MODE, &awbMode, 1);

    uint8_t awbState = (uint8_t)r.awbState;
    cm.update(ANDROID_CONTROL_AWB_STATE, &awbState, 1);
}

namespace {

/* A region the frame did not use (outside the crop) is not reported. */
void reportRegion(CameraMetadata &cm, uint32_t tag,
                  const std::optional<mocha::camera::MeteringRegion> &r) {
    if (r) {
        const int32_t region[] = {r->rect.xMin, r->rect.yMin,
                                  r->rect.xMax, r->rect.yMax, r->weight};
        cm.update(tag, region, 5);
    } else if (cm.exists(tag)) {
        cm.erase(tag);
    }
}

} /* namespace */

void ResultMetadataBuilder::buildRegions(CameraMetadata &cm, const Core3AResult &r) {
    if (r.cropRegion) {
        const int32_t crop[] = {r.cropRegion->left, r.cropRegion->top,
                                r.cropRegion->width, r.cropRegion->height};
        cm.update(ANDROID_SCALER_CROP_REGION, crop, 4);
    }
    reportRegion(cm, ANDROID_CONTROL_AE_REGIONS, r.aeRegion);
    reportRegion(cm, ANDROID_CONTROL_AF_REGIONS, r.afRegion);
    /* AWB takes no region (MAX_REGIONS): one a request sends anyway is
     * not reported. */
    if (cm.exists(ANDROID_CONTROL_AWB_REGIONS))
        cm.erase(ANDROID_CONTROL_AWB_REGIONS);
}

void ResultMetadataBuilder::buildFinal(CameraMetadata &cm, const Core3AResult &r,
                                        int64_t timestampNs, uint32_t frameNumber,
                                        float focalLengthMm) {
    buildBase(cm, timestampNs, frameNumber, focalLengthMm);
    buildAf  (cm, r);
    buildAe  (cm, r);
    buildAwb (cm, r);
    buildRegions(cm, r);
}

}; /* namespace android */
