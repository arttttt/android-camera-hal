#define LOG_TAG "Cam-StaticMeta"

#include "CameraStaticMetadata.h"

#include <stdint.h>
#include <sys/user.h>

#include <hardware/camera3.h>
#include <hardware/camera_common.h>
#include <camera/CameraMetadata.h>
#include <utils/misc.h>

#include "V4l2Device.h"
#include "sensor/SensorTuning.h"

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

/* Sensor + lens physical/geometric attributes. Pixel array size is
 * queried from the running V4L2 device; physical size / focal length /
 * minimum focus distance come from SensorTuning's `module` block
 * (per-module datasheet values). Fall back to aspect-correct fakes
 * when tuning isn't loaded so the framework doesn't reject the HAL. */
void writeSensorInfo(CameraMetadata &cm, V4l2Device *dev,
                     const SensorTuning *tuning, int facing) {
    auto sensorRes = dev->sensorResolution();

    float physSize[2];
    float focalLength;
    float minFocusDiopters;
    if (tuning && tuning->isLoaded()) {
        const auto &m = tuning->module();
        physSize[0]      = m.physicalSizeMm[0];
        physSize[1]      = m.physicalSizeMm[1];
        focalLength      = m.focalLengthMm;
        minFocusDiopters = m.minFocusDistanceDiopters;
    } else {
        /* Aspect-correct fallback so framework sees a coherent sensor
         * area when tuning failed to load. */
        physSize[0]      = 5.0f;
        physSize[1]      = 5.0f * (float)sensorRes.height / (float)sensorRes.width;
        focalLength      = 3.30f;
        minFocusDiopters = (facing == CAMERA_FACING_BACK) ? 10.0f : 0.0f;
    }

    cm.update(ANDROID_SENSOR_INFO_PHYSICAL_SIZE, physSize, 2);
    cm.update(ANDROID_LENS_INFO_AVAILABLE_FOCAL_LENGTHS, &focalLength, 1);
    cm.update(ANDROID_LENS_INFO_MINIMUM_FOCUS_DISTANCE, &minFocusDiopters, 1);

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

/* AE_AVAILABLE_TARGET_FPS_RANGES from VIDIOC_ENUM_FRAMEINTERVALS.
 *
 * For every FPS value the driver advertises across any supported
 * resolution, emit a constant-rate range `[fps, fps]` plus a variable
 * range `[kFpsLowerFloor, fps]` that lets AE drop the rate in low
 * light up to `fps`. Both forms are required by camera2 apps:
 * constants for video / ZSL, variable for preview AE flexibility.
 *
 * Falls back to {[15, 30], [30, 30]} if the driver reports no
 * intervals — same as the pre-honest hardcoded list, minus the
 * useless [15, 15] entry. Capped at `kFpsMaxRanges` pairs to bound
 * the metadata blob size when a sensor advertises many discrete
 * rates. */
constexpr int32_t kFpsLowerFloor = 15;
constexpr size_t  kFpsMaxRanges  = 16;

void writeAvailableFpsRanges(CameraMetadata &cm, V4l2Device *dev) {
    const Vector<int32_t> &fpsList = dev->availableFps();

    int32_t ranges[kFpsMaxRanges * 2];
    size_t  count  = 0;

    auto emit = [&](int32_t lo, int32_t hi) {
        if (count >= kFpsMaxRanges) return;
        ranges[2 * count + 0] = lo;
        ranges[2 * count + 1] = hi;
        ++count;
    };

    if (fpsList.isEmpty()) {
        emit(kFpsLowerFloor, 30);
        emit(30, 30);
    } else {
        for (size_t i = 0; i < fpsList.size(); ++i) {
            int32_t fps = fpsList[i];
            if (fps > kFpsLowerFloor) emit(kFpsLowerFloor, fps);
            emit(fps, fps);
        }
    }

    cm.update(ANDROID_CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES, ranges,
              count * 2);
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
        1  /* AF — single region for tap-to-focus */
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

    /* Capability sets the HAL implements. BACKWARD_COMPATIBLE is the
     * minimum required for any non-LEGACY hardware level — its
     * absence makes Camera2 service treat the HAL as LEGACY
     * regardless of SUPPORTED_HARDWARE_LEVEL, which is what kept
     * apps like Open Camera from offering the Camera2 toggle.
     * MANUAL_SENSOR / RAW / BURST are not yet claimed: real manual
     * exposure path needs a pre-capture-trigger contract we don't
     * implement, RAW is Phase 3 of B1, and BURST has no targeted
     * timing guarantees. */
    static const uint8_t requestCaps[] = {
        ANDROID_REQUEST_AVAILABLE_CAPABILITIES_BACKWARD_COMPATIBLE,
    };
    cm.update(ANDROID_REQUEST_AVAILABLE_CAPABILITIES,
              requestCaps, NELEM(requestCaps));

    /* Concurrent output stream caps. Produce-once landed in Tier 3 PR 7
     * so RGBA preview + YUV video are both non-stalling on the GPU side,
     * and the BLOB encode runs on JpegWorker concurrently with
     * pipeline submits — counts as one stalling stream because libjpeg
     * holds the JPEG snapshot ring slot until done. RAW and reprocess
     * are unimplemented; both set to 0 so apps don't try.
     *
     * Android 7.1.2 carries a single 3-element MAX_NUM_OUTPUT_STREAMS
     * array indexed [RAW, PROCESSED, PROCESSED_STALLING]. API 26+
     * split the same data into three scalar tags. Detect the split
     * by symbol presence so the same TU compiles against either. */
#ifdef ANDROID_REQUEST_MAX_NUM_OUTPUT_RAW
    static const int32_t maxOutputRaw      = 0;
    static const int32_t maxOutputProc     = 2;
    static const int32_t maxOutputStalling = 1;
    cm.update(ANDROID_REQUEST_MAX_NUM_OUTPUT_RAW,
              &maxOutputRaw,      1);
    cm.update(ANDROID_REQUEST_MAX_NUM_OUTPUT_PROC,
              &maxOutputProc,     1);
    cm.update(ANDROID_REQUEST_MAX_NUM_OUTPUT_PROC_STALLING,
              &maxOutputStalling, 1);
#else
    static const int32_t maxOutputStreams[] = {
        0, /* RAW                  */
        2, /* PROCESSED             */
        1, /* PROCESSED_STALLING    */
    };
    cm.update(ANDROID_REQUEST_MAX_NUM_OUTPUT_STREAMS,
              maxOutputStreams, NELEM(maxOutputStreams));
#endif

    static const int32_t maxInputStreams = 0;
    cm.update(ANDROID_REQUEST_MAX_NUM_INPUT_STREAMS,
              &maxInputStreams, 1);

    /* Frames between a request landing and its result reflecting the
     * applied controls. Exposure / gain go through DelayedControls
     * (2 frames on IMX179 + OV5693) but other controls (AE_MODE, AF
     * trigger) latch immediately. UNKNOWN advertises the worst case
     * without lying about per-frame controls. */
    static const int32_t syncMaxLatency = ANDROID_SYNC_MAX_LATENCY_UNKNOWN;
    cm.update(ANDROID_SYNC_MAX_LATENCY, &syncMaxLatency, 1);
}

/* Per-tag enumeration the framework / CameraX feature probes expect.
 * Strict subset of Camera2 keys this HAL actually consumes / writes —
 * advertising keys we do not handle would lead apps to set values we
 * silently drop. Three arrays must agree with what the request /
 * result builders populate; CHARACTERISTICS lists every key set in
 * the build() function below, including the AVAILABLE_*_KEYS arrays
 * themselves (Camera2 contract requires self-listing). */
void writeAvailableKeys(CameraMetadata &cm) {
    static const int32_t requestKeys[] = {
        ANDROID_CONTROL_AE_ANTIBANDING_MODE,
        ANDROID_CONTROL_AE_EXPOSURE_COMPENSATION,
        ANDROID_CONTROL_AE_LOCK,
        ANDROID_CONTROL_AE_MODE,
        ANDROID_CONTROL_AE_REGIONS,
        ANDROID_CONTROL_AE_TARGET_FPS_RANGE,
        ANDROID_CONTROL_AF_MODE,
        ANDROID_CONTROL_AF_REGIONS,
        ANDROID_CONTROL_AF_TRIGGER,
        ANDROID_CONTROL_AWB_LOCK,
        ANDROID_CONTROL_AWB_MODE,
        ANDROID_CONTROL_AWB_REGIONS,
        ANDROID_CONTROL_CAPTURE_INTENT,
        ANDROID_CONTROL_EFFECT_MODE,
        ANDROID_CONTROL_MODE,
        ANDROID_CONTROL_SCENE_MODE,
        ANDROID_CONTROL_VIDEO_STABILIZATION_MODE,
        ANDROID_JPEG_GPS_COORDINATES,
        ANDROID_JPEG_GPS_PROCESSING_METHOD,
        ANDROID_JPEG_GPS_TIMESTAMP,
        ANDROID_JPEG_ORIENTATION,
        ANDROID_JPEG_QUALITY,
        ANDROID_JPEG_THUMBNAIL_QUALITY,
        ANDROID_JPEG_THUMBNAIL_SIZE,
        ANDROID_LENS_FOCUS_DISTANCE,
        ANDROID_REQUEST_ID,
        ANDROID_SCALER_CROP_REGION,
        ANDROID_SENSOR_EXPOSURE_TIME,
        ANDROID_SENSOR_SENSITIVITY,
        ANDROID_STATISTICS_FACE_DETECT_MODE,
        ANDROID_STATISTICS_HISTOGRAM_MODE,
        ANDROID_STATISTICS_SHARPNESS_MAP_MODE,
    };
    cm.update(ANDROID_REQUEST_AVAILABLE_REQUEST_KEYS,
              requestKeys, NELEM(requestKeys));

    /* Result keys = round-tripped request keys (any key the app set
     * lands back unchanged in cm) plus result-only keys the
     * ResultMetadataBuilder writes per frame. */
    static const int32_t resultKeys[] = {
        /* Round-tripped request side */
        ANDROID_CONTROL_AE_ANTIBANDING_MODE,
        ANDROID_CONTROL_AE_EXPOSURE_COMPENSATION,
        ANDROID_CONTROL_AE_LOCK,
        ANDROID_CONTROL_AE_MODE,
        ANDROID_CONTROL_AE_REGIONS,
        ANDROID_CONTROL_AE_TARGET_FPS_RANGE,
        ANDROID_CONTROL_AF_MODE,
        ANDROID_CONTROL_AF_REGIONS,
        ANDROID_CONTROL_AF_TRIGGER,
        ANDROID_CONTROL_AWB_LOCK,
        ANDROID_CONTROL_AWB_MODE,
        ANDROID_CONTROL_AWB_REGIONS,
        ANDROID_CONTROL_CAPTURE_INTENT,
        ANDROID_CONTROL_EFFECT_MODE,
        ANDROID_CONTROL_MODE,
        ANDROID_CONTROL_SCENE_MODE,
        ANDROID_CONTROL_VIDEO_STABILIZATION_MODE,
        ANDROID_JPEG_GPS_COORDINATES,
        ANDROID_JPEG_GPS_PROCESSING_METHOD,
        ANDROID_JPEG_GPS_TIMESTAMP,
        ANDROID_JPEG_ORIENTATION,
        ANDROID_JPEG_QUALITY,
        ANDROID_JPEG_THUMBNAIL_QUALITY,
        ANDROID_JPEG_THUMBNAIL_SIZE,
        ANDROID_REQUEST_ID,
        ANDROID_SCALER_CROP_REGION,
        ANDROID_STATISTICS_FACE_DETECT_MODE,
        ANDROID_STATISTICS_HISTOGRAM_MODE,
        ANDROID_STATISTICS_SHARPNESS_MAP_MODE,
        /* Result-only — written per-frame by ResultMetadataBuilder */
        ANDROID_CONTROL_AE_STATE,
        ANDROID_CONTROL_AF_STATE,
        ANDROID_CONTROL_AWB_STATE,
        ANDROID_LENS_APERTURE,
        ANDROID_LENS_FOCAL_LENGTH,
        ANDROID_LENS_FOCUS_DISTANCE,
        ANDROID_SENSOR_EXPOSURE_TIME,
        ANDROID_SENSOR_FRAME_DURATION,
        ANDROID_SENSOR_SENSITIVITY,
        ANDROID_SENSOR_TIMESTAMP,
        ANDROID_SYNC_FRAME_NUMBER,
    };
    cm.update(ANDROID_REQUEST_AVAILABLE_RESULT_KEYS,
              resultKeys, NELEM(resultKeys));

    static const int32_t characteristicsKeys[] = {
        ANDROID_CONTROL_AE_AVAILABLE_ANTIBANDING_MODES,
        ANDROID_CONTROL_AE_AVAILABLE_MODES,
        ANDROID_CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES,
        ANDROID_CONTROL_AE_COMPENSATION_RANGE,
        ANDROID_CONTROL_AE_COMPENSATION_STEP,
        ANDROID_CONTROL_AF_AVAILABLE_MODES,
        ANDROID_CONTROL_AVAILABLE_EFFECTS,
        ANDROID_CONTROL_AVAILABLE_SCENE_MODES,
        ANDROID_CONTROL_AVAILABLE_VIDEO_STABILIZATION_MODES,
        ANDROID_CONTROL_AWB_AVAILABLE_MODES,
        ANDROID_CONTROL_MAX_REGIONS,
        ANDROID_FLASH_INFO_AVAILABLE,
        ANDROID_INFO_SUPPORTED_HARDWARE_LEVEL,
        ANDROID_JPEG_AVAILABLE_THUMBNAIL_SIZES,
        ANDROID_JPEG_MAX_SIZE,
        ANDROID_LENS_FACING,
        ANDROID_LENS_INFO_AVAILABLE_FOCAL_LENGTHS,
        ANDROID_LENS_INFO_MINIMUM_FOCUS_DISTANCE,
        ANDROID_REQUEST_AVAILABLE_CAPABILITIES,
        ANDROID_REQUEST_AVAILABLE_CHARACTERISTICS_KEYS,
        ANDROID_REQUEST_AVAILABLE_REQUEST_KEYS,
        ANDROID_REQUEST_AVAILABLE_RESULT_KEYS,
        ANDROID_REQUEST_MAX_NUM_INPUT_STREAMS,
#ifdef ANDROID_REQUEST_MAX_NUM_OUTPUT_RAW
        ANDROID_REQUEST_MAX_NUM_OUTPUT_PROC,
        ANDROID_REQUEST_MAX_NUM_OUTPUT_PROC_STALLING,
        ANDROID_REQUEST_MAX_NUM_OUTPUT_RAW,
#else
        ANDROID_REQUEST_MAX_NUM_OUTPUT_STREAMS,
#endif
        ANDROID_REQUEST_PARTIAL_RESULT_COUNT,
        ANDROID_REQUEST_PIPELINE_MAX_DEPTH,
        ANDROID_SCALER_AVAILABLE_FORMATS,
        ANDROID_SCALER_AVAILABLE_JPEG_MIN_DURATIONS,
        ANDROID_SCALER_AVAILABLE_JPEG_SIZES,
        ANDROID_SCALER_AVAILABLE_MAX_DIGITAL_ZOOM,
        ANDROID_SCALER_AVAILABLE_MIN_FRAME_DURATIONS,
        ANDROID_SCALER_AVAILABLE_PROCESSED_MIN_DURATIONS,
        ANDROID_SCALER_AVAILABLE_PROCESSED_SIZES,
        ANDROID_SCALER_AVAILABLE_STALL_DURATIONS,
        ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS,
        ANDROID_SENSOR_INFO_ACTIVE_ARRAY_SIZE,
        ANDROID_SENSOR_INFO_EXPOSURE_TIME_RANGE,
        ANDROID_SENSOR_INFO_PHYSICAL_SIZE,
        ANDROID_SENSOR_INFO_PIXEL_ARRAY_SIZE,
        ANDROID_SENSOR_INFO_SENSITIVITY_RANGE,
        ANDROID_SENSOR_MAX_ANALOG_SENSITIVITY,
        ANDROID_SENSOR_ORIENTATION,
        ANDROID_STATISTICS_INFO_MAX_FACE_COUNT,
        ANDROID_SYNC_MAX_LATENCY,
    };
    cm.update(ANDROID_REQUEST_AVAILABLE_CHARACTERISTICS_KEYS,
              characteristicsKeys, NELEM(characteristicsKeys));
}

} /* namespace */

camera_metadata_t *CameraStaticMetadata::build(V4l2Device *dev, int facing,
                                                const SensorTuning *tuning,
                                                size_t *jpegBufferSize) {
    CameraMetadata cm;
    writeSensorInfo         (cm, dev, tuning, facing);
    writeScalerConfigs      (cm, dev);
    writeJpegInfo           (cm, dev, jpegBufferSize);
    writeSensorRanges       (cm);
    writeAvailableFpsRanges (cm, dev);
    writeControlInfo        (cm, facing);
    writeHalInfo            (cm);
    writeAvailableKeys      (cm);
    return cm.release();
}

}; /* namespace android */
