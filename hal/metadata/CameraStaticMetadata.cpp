#define LOG_TAG "Cam-StaticMeta"

#include "CameraStaticMetadata.h"

#include <stdint.h>
#include <sys/user.h>

#include <hardware/camera3.h>
#include <hardware/camera_common.h>
#include <camera/CameraMetadata.h>
#include <utils/misc.h>

#include "V4l2Device.h"

namespace android {

namespace {

/* Single-buffered synchronous pipeline: processCaptureRequest holds
 * the camera mutex end-to-end and emits one result per call. Raise
 * both once the request-queue refactor (Tier 3) introduces real
 * pipelining. */
constexpr uint8_t  kPipelineMaxDepth   = 1;
constexpr int32_t  kPartialResultCount = 1;

/* Conservative 30 fps cap when the driver does not report a framerate
 * for a given mode — better for the framework than a 60 fps lie. */
constexpr int64_t  kFallbackMinFrameDurationNs = 1000000000LL / 30;

/* Sensor + lens physical/geometric attributes. Values that come from
 * the running V4L2 device (pixel array size) read `dev` directly;
 * everything else is a placeholder until per-sensor tuning lands
 * (Tier 2). */
void writeSensorInfo(CameraMetadata &cm, V4l2Device *dev, int facing) {
    auto sensorRes = dev->sensorResolution();

    /* fake, but valid aspect ratio */
    const float sensorInfoPhysicalSize[] = {
        5.0f,
        5.0f * (float)sensorRes.height / (float)sensorRes.width
    };
    cm.update(ANDROID_SENSOR_INFO_PHYSICAL_SIZE, sensorInfoPhysicalSize, NELEM(sensorInfoPhysicalSize));

    /* fake */
    static const float lensInfoAvailableFocalLengths[] = {3.30f};
    cm.update(ANDROID_LENS_INFO_AVAILABLE_FOCAL_LENGTHS, lensInfoAvailableFocalLengths, NELEM(lensInfoAvailableFocalLengths));

    /* Minimum focus distance > 0 tells framework this lens can focus */
    if (facing == CAMERA_FACING_BACK) {
        static const float lensInfoMinFocusDistance = 10.0f; /* 1/10m = 10cm macro */
        cm.update(ANDROID_LENS_INFO_MINIMUM_FOCUS_DISTANCE, &lensInfoMinFocusDistance, 1);
    } else {
        static const float lensInfoMinFocusDistance = 0.0f; /* fixed focus */
        cm.update(ANDROID_LENS_INFO_MINIMUM_FOCUS_DISTANCE, &lensInfoMinFocusDistance, 1);
    }

    const uint8_t lensFacing = (facing == CAMERA_FACING_FRONT)
        ? ANDROID_LENS_FACING_FRONT : ANDROID_LENS_FACING_BACK;
    cm.update(ANDROID_LENS_FACING, &lensFacing, 1);

    const int32_t sensorInfoPixelArraySize[] = {
        (int32_t)sensorRes.width,
        (int32_t)sensorRes.height
    };
    cm.update(ANDROID_SENSOR_INFO_PIXEL_ARRAY_SIZE, sensorInfoPixelArraySize, NELEM(sensorInfoPixelArraySize));

    const int32_t sensorInfoActiveArraySize[] = {
        0,                          0,
        (int32_t)sensorRes.width,   (int32_t)sensorRes.height
    };
    cm.update(ANDROID_SENSOR_INFO_ACTIVE_ARRAY_SIZE, sensorInfoActiveArraySize, NELEM(sensorInfoActiveArraySize));

    const int32_t sensorOrientation = (facing == CAMERA_FACING_FRONT) ? 270 : 90;
    cm.update(ANDROID_SENSOR_ORIENTATION, &sensorOrientation, 1);
}

/* Resolution × format tables + per-mode durations. This is the bulk of
 * the characteristics — every supported (format, width, height) pair
 * needs to be enumerated here so CameraX / Camera2 can pick a stream. */
void writeScalerConfigs(CameraMetadata &cm, V4l2Device *dev) {
    auto &resolutions = dev->availableResolutions();
    auto &previewResolutions = resolutions;

    static const int32_t scalerAvailableFormats[] = {
        HAL_PIXEL_FORMAT_RGBA_8888,
        HAL_PIXEL_FORMAT_YCbCr_420_888,
        HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED,
        /* Non-preview one, must be last - see following code */
        HAL_PIXEL_FORMAT_BLOB
    };
    cm.update(ANDROID_SCALER_AVAILABLE_FORMATS, scalerAvailableFormats, NELEM(scalerAvailableFormats));

    /* Only for HAL_PIXEL_FORMAT_BLOB */
    const size_t mainStreamConfigsCount = resolutions.size();
    /* For all other supported pixel formats */
    const size_t previewStreamConfigsCount = previewResolutions.size() * (NELEM(scalerAvailableFormats) - 1);
    const size_t streamConfigsCount = mainStreamConfigsCount + previewStreamConfigsCount;

    int32_t scalerAvailableStreamConfigurations[streamConfigsCount * 4];
    int64_t scalerAvailableMinFrameDurations[streamConfigsCount * 4];

    int32_t scalerAvailableProcessedSizes[previewResolutions.size() * 2];
    int64_t scalerAvailableProcessedMinDurations[previewResolutions.size()];
    int32_t scalerAvailableJpegSizes[resolutions.size() * 2];
    int64_t scalerAvailableJpegMinDurations[resolutions.size()];

    size_t i4 = 0;
    size_t i2 = 0;
    size_t i1 = 0;
    /* Main stream configurations */
    for(size_t resId = 0; resId < resolutions.size(); ++resId) {
        const auto &r = resolutions[resId];
        int64_t minDur = dev->minFrameDurationNs(r.width, r.height);
        if (minDur <= 0) minDur = kFallbackMinFrameDurationNs;

        scalerAvailableStreamConfigurations[i4 + 0] = HAL_PIXEL_FORMAT_BLOB;
        scalerAvailableStreamConfigurations[i4 + 1] = (int32_t)r.width;
        scalerAvailableStreamConfigurations[i4 + 2] = (int32_t)r.height;
        scalerAvailableStreamConfigurations[i4 + 3] = ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT;

        scalerAvailableMinFrameDurations[i4 + 0] = HAL_PIXEL_FORMAT_BLOB;
        scalerAvailableMinFrameDurations[i4 + 1] = (int32_t)r.width;
        scalerAvailableMinFrameDurations[i4 + 2] = (int32_t)r.height;
        scalerAvailableMinFrameDurations[i4 + 3] = minDur;

        scalerAvailableJpegSizes[i2 + 0] = (int32_t)r.width;
        scalerAvailableJpegSizes[i2 + 1] = (int32_t)r.height;

        scalerAvailableJpegMinDurations[i1] = minDur;

        i4 += 4;
        i2 += 2;
        i1 += 1;
    }
    i2 = 0;
    i1 = 0;
    /* Preview stream configurations */
    for(size_t resId = 0; resId < previewResolutions.size(); ++resId) {
        const auto &r = previewResolutions[resId];
        int64_t minDur = dev->minFrameDurationNs(r.width, r.height);
        if (minDur <= 0) minDur = kFallbackMinFrameDurationNs;

        for(size_t fmtId = 0; fmtId < NELEM(scalerAvailableFormats) - 1; ++fmtId) {
            scalerAvailableStreamConfigurations[i4 + 0] = scalerAvailableFormats[fmtId];
            scalerAvailableStreamConfigurations[i4 + 1] = (int32_t)r.width;
            scalerAvailableStreamConfigurations[i4 + 2] = (int32_t)r.height;
            scalerAvailableStreamConfigurations[i4 + 3] = ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT;

            scalerAvailableMinFrameDurations[i4 + 0] = scalerAvailableFormats[fmtId];
            scalerAvailableMinFrameDurations[i4 + 1] = (int32_t)r.width;
            scalerAvailableMinFrameDurations[i4 + 2] = (int32_t)r.height;
            scalerAvailableMinFrameDurations[i4 + 3] = minDur;

            i4 += 4;
        }
        scalerAvailableProcessedSizes[i2 + 0] = (int32_t)r.width;
        scalerAvailableProcessedSizes[i2 + 1] = (int32_t)r.height;

        scalerAvailableProcessedMinDurations[i1] = minDur;

        i2 += 2;
        i1 += 1;
    }
    cm.update(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS, scalerAvailableStreamConfigurations, (size_t)NELEM(scalerAvailableStreamConfigurations));
    cm.update(ANDROID_SCALER_AVAILABLE_MIN_FRAME_DURATIONS, scalerAvailableMinFrameDurations, (size_t)NELEM(scalerAvailableMinFrameDurations));
    /* Probably fake */
    cm.update(ANDROID_SCALER_AVAILABLE_STALL_DURATIONS, scalerAvailableMinFrameDurations, (size_t)NELEM(scalerAvailableMinFrameDurations));
    cm.update(ANDROID_SCALER_AVAILABLE_JPEG_SIZES, scalerAvailableJpegSizes, (size_t)NELEM(scalerAvailableJpegSizes));
    cm.update(ANDROID_SCALER_AVAILABLE_JPEG_MIN_DURATIONS, scalerAvailableJpegMinDurations, (size_t)NELEM(scalerAvailableJpegMinDurations));
    cm.update(ANDROID_SCALER_AVAILABLE_PROCESSED_SIZES, scalerAvailableProcessedSizes, (size_t)NELEM(scalerAvailableProcessedSizes));
    cm.update(ANDROID_SCALER_AVAILABLE_PROCESSED_MIN_DURATIONS, scalerAvailableProcessedMinDurations, (size_t)NELEM(scalerAvailableProcessedMinDurations));

    static const float scalerAvailableMaxDigitalZoom = 4;
    cm.update(ANDROID_SCALER_AVAILABLE_MAX_DIGITAL_ZOOM, &scalerAvailableMaxDigitalZoom, 1);
}

/* JPEG buffer sizing + thumbnail options. Writes *jpegBufferSize so the
 * caller can use it for BLOB allocations in configureStreams. */
void writeJpegInfo(CameraMetadata &cm, V4l2Device *dev, size_t *jpegBufferSize) {
    auto sensorRes = dev->sensorResolution();

    /* ~1.1 byte/px typical JPEG, use 2 byte/px for safety margin */
    size_t jpegBuf = sensorRes.width * sensorRes.height * 2 + sizeof(camera3_jpeg_blob);
    jpegBuf = (jpegBuf + PAGE_SIZE - 1u) & ~(PAGE_SIZE - 1u);
    *jpegBufferSize = jpegBuf;
    const int32_t jpegMaxSize = (int32_t)jpegBuf;
    cm.update(ANDROID_JPEG_MAX_SIZE, &jpegMaxSize, 1);

    static const int32_t jpegAvailableThumbnailSizes[] = {
        0, 0,
        320, 240
    };
    cm.update(ANDROID_JPEG_AVAILABLE_THUMBNAIL_SIZES, jpegAvailableThumbnailSizes, NELEM(jpegAvailableThumbnailSizes));
}

/* Declared sensor operating ranges — exposure time, ISO, analog gain
 * ceiling. Fixed ranges for now; per-sensor tuning (Tier 2) will pull
 * these from JSON. */
void writeSensorRanges(CameraMetadata &cm) {
    /* Exposure time: 0.1ms to 200ms */
    static const int64_t sensorExposureTimeRange[] = { 100000LL, 200000000LL };
    cm.update(ANDROID_SENSOR_INFO_EXPOSURE_TIME_RANGE, sensorExposureTimeRange, NELEM(sensorExposureTimeRange));

    /* ISO sensitivity: 100 to 3200 */
    static const int32_t sensorSensitivityRange[] = { 100, 3200 };
    cm.update(ANDROID_SENSOR_INFO_SENSITIVITY_RANGE, sensorSensitivityRange, NELEM(sensorSensitivityRange));

    static const int32_t sensorMaxAnalogSensitivity = 1600;
    cm.update(ANDROID_SENSOR_MAX_ANALOG_SENSITIVITY, &sensorMaxAnalogSensitivity, 1);
}

/* 3A capabilities + feature flags (flash, stats, scene modes, effects,
 * stabilization). Everything that tells the framework "what we support"
 * outside of resolutions/ranges. */
void writeControlInfo(CameraMetadata &cm, int facing) {
    static const uint8_t flashInfoAvailable = ANDROID_FLASH_INFO_AVAILABLE_FALSE;
    cm.update(ANDROID_FLASH_INFO_AVAILABLE, &flashInfoAvailable, 1);

    static const uint8_t statisticsFaceDetectModes[] = {
        ANDROID_STATISTICS_FACE_DETECT_MODE_OFF
    };
    cm.update(ANDROID_STATISTICS_FACE_DETECT_MODE, statisticsFaceDetectModes, NELEM(statisticsFaceDetectModes));

    static const int32_t statisticsInfoMaxFaceCount = 0;
    cm.update(ANDROID_STATISTICS_INFO_MAX_FACE_COUNT, &statisticsInfoMaxFaceCount, 1);

    static const uint8_t controlAvailableSceneModes[] = {
        ANDROID_CONTROL_SCENE_MODE_DISABLED
    };
    cm.update(ANDROID_CONTROL_AVAILABLE_SCENE_MODES, controlAvailableSceneModes, NELEM(controlAvailableSceneModes));

    static const uint8_t controlAvailableEffects[] = {
            ANDROID_CONTROL_EFFECT_MODE_OFF
    };
    cm.update(ANDROID_CONTROL_AVAILABLE_EFFECTS, controlAvailableEffects, NELEM(controlAvailableEffects));

    static const int32_t controlMaxRegions[] = {
        0, /* AE */
        0, /* AWB */
        0  /* AF */
    };
    cm.update(ANDROID_CONTROL_MAX_REGIONS, controlMaxRegions, NELEM(controlMaxRegions));

    static const uint8_t controlAeAvailableModes[] = {
            ANDROID_CONTROL_AE_MODE_OFF
    };
    cm.update(ANDROID_CONTROL_AE_AVAILABLE_MODES, controlAeAvailableModes, NELEM(controlAeAvailableModes));

    static const camera_metadata_rational controlAeCompensationStep = {1, 3};
    cm.update(ANDROID_CONTROL_AE_COMPENSATION_STEP, &controlAeCompensationStep, 1);

    int32_t controlAeCompensationRange[] = {-9, 9};
    cm.update(ANDROID_CONTROL_AE_COMPENSATION_RANGE, controlAeCompensationRange, NELEM(controlAeCompensationRange));

    static const int32_t controlAeAvailableTargetFpsRanges[] = {
        15, 15,
        15, 30,
        30, 30,
    };
    cm.update(ANDROID_CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES, controlAeAvailableTargetFpsRanges, NELEM(controlAeAvailableTargetFpsRanges));

    static const uint8_t controlAeAvailableAntibandingModes[] = {
            ANDROID_CONTROL_AE_ANTIBANDING_MODE_OFF
    };
    cm.update(ANDROID_CONTROL_AE_AVAILABLE_ANTIBANDING_MODES, controlAeAvailableAntibandingModes, NELEM(controlAeAvailableAntibandingModes));

    static const uint8_t controlAwbAvailableModes[] = {
            ANDROID_CONTROL_AWB_MODE_AUTO,
            ANDROID_CONTROL_AWB_MODE_OFF
    };
    cm.update(ANDROID_CONTROL_AWB_AVAILABLE_MODES, controlAwbAvailableModes, NELEM(controlAwbAvailableModes));

    if (facing == CAMERA_FACING_BACK) {
        uint8_t controlAfAvailableModes[] = {
            ANDROID_CONTROL_AF_MODE_OFF,
            ANDROID_CONTROL_AF_MODE_AUTO,
            ANDROID_CONTROL_AF_MODE_MACRO,
            ANDROID_CONTROL_AF_MODE_CONTINUOUS_PICTURE,
        };
        cm.update(ANDROID_CONTROL_AF_AVAILABLE_MODES, controlAfAvailableModes, NELEM(controlAfAvailableModes));
    } else {
        uint8_t controlAfAvailableModes[] = { ANDROID_CONTROL_AF_MODE_OFF };
        cm.update(ANDROID_CONTROL_AF_AVAILABLE_MODES, controlAfAvailableModes, 1);
    }

    static const uint8_t controlAvailableVideoStabilizationModes[] = {
            ANDROID_CONTROL_VIDEO_STABILIZATION_MODE_OFF
    };
    cm.update(ANDROID_CONTROL_AVAILABLE_VIDEO_STABILIZATION_MODES, controlAvailableVideoStabilizationModes, NELEM(controlAvailableVideoStabilizationModes));
}

/* HAL capability level + request-pipeline shape. Both constants go up
 * once the request-queue refactor (Tier 3) lands. */
void writeHalInfo(CameraMetadata &cm) {
    const uint8_t infoSupportedHardwareLevel = ANDROID_INFO_SUPPORTED_HARDWARE_LEVEL_LIMITED;
    cm.update(ANDROID_INFO_SUPPORTED_HARDWARE_LEVEL, &infoSupportedHardwareLevel, 1);

    cm.update(ANDROID_REQUEST_PIPELINE_MAX_DEPTH,   &kPipelineMaxDepth,   1);
    cm.update(ANDROID_REQUEST_PARTIAL_RESULT_COUNT, &kPartialResultCount, 1);
}

} /* namespace */

camera_metadata_t *CameraStaticMetadata::build(V4l2Device *dev, int facing,
                                                size_t *jpegBufferSize) {
    CameraMetadata cm;
    writeSensorInfo   (cm, dev, facing);
    writeScalerConfigs(cm, dev);
    writeJpegInfo     (cm, dev, jpegBufferSize);
    writeSensorRanges (cm);
    writeControlInfo  (cm, facing);
    writeHalInfo      (cm);
    return cm.release();
}

}; /* namespace android */
