#define LOG_TAG "Cam-StaticMeta"

#include "CameraStaticMetadata.h"

#include <stdint.h>
#include <string.h>
#include <sys/user.h>
#include <vector>
#include <linux/videodev2.h>

#include <hardware/camera3.h>
#include <hardware/camera_common.h>
#include "CameraMetadataCompat.h"
#include <utils/misc.h>


namespace android {

using mocha::camera::AeMode;
using mocha::camera::AfMode;
using mocha::camera::AwbMode;
using mocha::camera::Capability;
using mocha::camera::PixelFormat;

namespace {

/* What the core produces in Camera3's pixel formats. Preview's
 * IMPLEMENTATION_DEFINED is produced as RGBA (StreamConfig maps it),
 * so it is offered wherever RGBA is. */
int32_t halFormatOf(PixelFormat f) {
    switch (f) {
        case PixelFormat::Rgba8888: return HAL_PIXEL_FORMAT_RGBA_8888;
        case PixelFormat::Ycbcr420: return HAL_PIXEL_FORMAT_YCbCr_420_888;
        case PixelFormat::Blob:     return HAL_PIXEL_FORMAT_BLOB;
    }
    return HAL_PIXEL_FORMAT_RGBA_8888;
}

/* Sensor + lens physical/geometric attributes, from the capability. */
void writeSensorInfo(CameraMetadata &cm, const Capability &cap) {
    const float physSize[2] = { cap.sensor.physicalSizeMm[0], cap.sensor.physicalSizeMm[1] };
    const float focalLength      = cap.lens.focalLengthMm;
    const float minFocusDiopters = cap.lens.minFocusDistanceDiopters;
    cm.update(ANDROID_SENSOR_INFO_PHYSICAL_SIZE, physSize, 2);
    cm.update(ANDROID_LENS_INFO_AVAILABLE_FOCAL_LENGTHS, &focalLength, 1);
    cm.update(ANDROID_LENS_INFO_MINIMUM_FOCUS_DISTANCE, &minFocusDiopters, 1);

    const uint8_t lensFacing = cap.facingFront ? ANDROID_LENS_FACING_FRONT
                                               : ANDROID_LENS_FACING_BACK;
    cm.update(ANDROID_LENS_FACING, &lensFacing, 1);

    const int32_t pixelArraySize[] = {
        (int32_t)cap.sensor.pixelArray.width,
        (int32_t)cap.sensor.pixelArray.height
    };
    cm.update(ANDROID_SENSOR_INFO_PIXEL_ARRAY_SIZE, pixelArraySize, NELEM(pixelArraySize));

    const int32_t activeArraySize[] = {
        cap.sensor.activeArray.left,
        cap.sensor.activeArray.top,
        cap.sensor.activeArray.width,
        cap.sensor.activeArray.height
    };
    cm.update(ANDROID_SENSOR_INFO_ACTIVE_ARRAY_SIZE, activeArraySize, NELEM(activeArraySize));

    const int32_t sensorOrientation = cap.sensor.orientationDegrees;
    cm.update(ANDROID_SENSOR_ORIENTATION, &sensorOrientation, 1);
}

/* Resolution x format tables + per-mode durations, from the
 * capability's stream list: exactly the outputs the pipeline produces,
 * each at its fastest. The legacy per-format size lists (JPEG,
 * processed) are the same sizes. */
void writeScalerConfigs(CameraMetadata &cm, const Capability &cap) {
    static const int32_t scalerAvailableFormats[] = {
        HAL_PIXEL_FORMAT_RGBA_8888,
        HAL_PIXEL_FORMAT_YCbCr_420_888,
        HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED,
        HAL_PIXEL_FORMAT_BLOB
    };
    cm.update(ANDROID_SCALER_AVAILABLE_FORMATS, scalerAvailableFormats, NELEM(scalerAvailableFormats));

    std::vector<int32_t> configs;
    std::vector<int64_t> minDurations;
    std::vector<int64_t> stallDurations;
    std::vector<int32_t> jpegSizes;
    std::vector<int64_t> jpegMinDurations;
    std::vector<int32_t> processedSizes;
    std::vector<int64_t> processedMinDurations;
    auto add = [&](int32_t format, const Capability::StreamConfig &sc) {
        const int32_t w = (int32_t)sc.size.width;
        const int32_t h = (int32_t)sc.size.height;
        configs.insert(configs.end(), {format, w, h,
                ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT});
        minDurations.insert(minDurations.end(), {format, w, h, sc.minFrameDurationNs});
        stallDurations.insert(stallDurations.end(), {format, w, h, sc.stallDurationNs});
    };
    for (const Capability::StreamConfig &sc : cap.streams) {
        add(halFormatOf(sc.format), sc);
        if (sc.format == PixelFormat::Rgba8888) {
            add(HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED, sc);
            processedSizes.insert(processedSizes.end(),
                                  {(int32_t)sc.size.width, (int32_t)sc.size.height});
            processedMinDurations.push_back(sc.minFrameDurationNs);
        } else if (sc.format == PixelFormat::Blob) {
            jpegSizes.insert(jpegSizes.end(),
                             {(int32_t)sc.size.width, (int32_t)sc.size.height});
            jpegMinDurations.push_back(sc.minFrameDurationNs);
        }
    }
    cm.update(ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS, configs.data(), configs.size());
    cm.update(ANDROID_SCALER_AVAILABLE_MIN_FRAME_DURATIONS, minDurations.data(), minDurations.size());
    cm.update(ANDROID_SCALER_AVAILABLE_STALL_DURATIONS, stallDurations.data(), stallDurations.size());
    cm.update(ANDROID_SCALER_AVAILABLE_JPEG_SIZES, jpegSizes.data(), jpegSizes.size());
    cm.update(ANDROID_SCALER_AVAILABLE_JPEG_MIN_DURATIONS, jpegMinDurations.data(), jpegMinDurations.size());
    cm.update(ANDROID_SCALER_AVAILABLE_PROCESSED_SIZES, processedSizes.data(), processedSizes.size());
    cm.update(ANDROID_SCALER_AVAILABLE_PROCESSED_MIN_DURATIONS, processedMinDurations.data(), processedMinDurations.size());

    const float maxDigitalZoom = cap.maxDigitalZoom;
    cm.update(ANDROID_SCALER_AVAILABLE_MAX_DIGITAL_ZOOM, &maxDigitalZoom, 1);
}

/* JPEG buffer sizing + thumbnail options. */
void writeJpegInfo(CameraMetadata &cm, const Capability &cap) {
    const int32_t jpegMaxSize = (int32_t)cap.jpeg.maxSize;
    cm.update(ANDROID_JPEG_MAX_SIZE, &jpegMaxSize, 1);

    std::vector<int32_t> thumbnailSizes;
    for (const auto &t : cap.jpeg.thumbnailSizes)
        thumbnailSizes.insert(thumbnailSizes.end(), {(int32_t)t.width, (int32_t)t.height});
    cm.update(ANDROID_JPEG_AVAILABLE_THUMBNAIL_SIZES, thumbnailSizes.data(), thumbnailSizes.size());
}

/* Declared sensor operating ranges -- exposure time + ISO sensitivity,
 * as the driver reported them. A range the driver could not give is
 * left out, and the framework will reject the HAL: surfacing the
 * breakage rather than lying about what a slider should offer. Past
 * the sensor's analog maximum the ISP's digital gain goes on (to
 * SensorConfig::digitalGainMax): the range reaches its top, and
 * MAX_ANALOG_SENSITIVITY says where the analog gain ends. */
void writeSensorRanges(CameraMetadata &cm, const Capability &cap) {
    if (cap.sensor.exposureMaxNs > 0) {
        const int64_t range[] = { cap.sensor.exposureMinNs, cap.sensor.exposureMaxNs };
        cm.update(ANDROID_SENSOR_INFO_EXPOSURE_TIME_RANGE, range, NELEM(range));
    }
    if (cap.sensor.maxFrameDurationNs > 0)
        cm.update(ANDROID_SENSOR_INFO_MAX_FRAME_DURATION,
                  &cap.sensor.maxFrameDurationNs, 1);
    if (cap.sensor.sensitivityMaxIso > 0) {
        const int32_t range[] = { cap.sensor.sensitivityMinIso, cap.sensor.sensitivityMaxIso };
        cm.update(ANDROID_SENSOR_INFO_SENSITIVITY_RANGE, range, NELEM(range));
        const int32_t maxAnalog = cap.sensor.maxAnalogIso;
        cm.update(ANDROID_SENSOR_MAX_ANALOG_SENSITIVITY, &maxAnalog, 1);
    }
}

void writeAvailableFpsRanges(CameraMetadata &cm, const Capability &cap) {
    std::vector<int32_t> ranges;
    for (const auto &r : cap.ae.targetFpsRanges)
        ranges.insert(ranges.end(), {r.min, r.max});
    cm.update(ANDROID_CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES, ranges.data(), ranges.size());
}

/* 3A capabilities + feature flags (flash, stats, scene modes, effects,
 * stabilization). Everything that tells the framework "what we support"
 * outside of resolutions/ranges. */
void writeControlInfo(CameraMetadata &cm, const Capability &cap) {
    static const uint8_t flashInfoAvailable = ANDROID_FLASH_INFO_AVAILABLE_FALSE;
    cm.update(ANDROID_FLASH_INFO_AVAILABLE, &flashInfoAvailable, 1);

    static const uint8_t statisticsFaceDetectModes[] = {
        ANDROID_STATISTICS_FACE_DETECT_MODE_OFF
    };
    cm.update(ANDROID_STATISTICS_INFO_AVAILABLE_FACE_DETECT_MODES,
              statisticsFaceDetectModes, NELEM(statisticsFaceDetectModes));

    static const int32_t statisticsInfoMaxFaceCount = 0;
    cm.update(ANDROID_STATISTICS_INFO_MAX_FACE_COUNT, &statisticsInfoMaxFaceCount, 1);

    static const uint8_t statisticsAvailableHotPixelMapModes[] = {
        ANDROID_STATISTICS_HOT_PIXEL_MAP_MODE_OFF
    };
    cm.update(ANDROID_STATISTICS_INFO_AVAILABLE_HOT_PIXEL_MAP_MODES,
              statisticsAvailableHotPixelMapModes,
              NELEM(statisticsAvailableHotPixelMapModes));

    static const uint8_t statisticsAvailableLensShadingMapModes[] = {
        ANDROID_STATISTICS_LENS_SHADING_MAP_MODE_OFF
    };
    cm.update(ANDROID_STATISTICS_INFO_AVAILABLE_LENS_SHADING_MAP_MODES,
              statisticsAvailableLensShadingMapModes,
              NELEM(statisticsAvailableLensShadingMapModes));

    static const uint8_t controlAvailableSceneModes[] = {
        ANDROID_CONTROL_SCENE_MODE_DISABLED
    };
    cm.update(ANDROID_CONTROL_AVAILABLE_SCENE_MODES, controlAvailableSceneModes, NELEM(controlAvailableSceneModes));

    static const uint8_t controlAvailableEffects[] = {
            ANDROID_CONTROL_EFFECT_MODE_OFF
    };
    cm.update(ANDROID_CONTROL_AVAILABLE_EFFECTS, controlAvailableEffects, NELEM(controlAvailableEffects));

    const int32_t controlMaxRegions[] = {
        cap.ae.maxRegions,  /* AE  */
        0,                  /* AWB */
        cap.af.maxRegions   /* AF -- tap-to-focus */
    };
    cm.update(ANDROID_CONTROL_MAX_REGIONS, controlMaxRegions, NELEM(controlMaxRegions));

    std::vector<uint8_t> aeModes;
    for (AeMode m : cap.ae.modes) aeModes.push_back((uint8_t)m);
    cm.update(ANDROID_CONTROL_AE_AVAILABLE_MODES, aeModes.data(), aeModes.size());

    const camera_metadata_rational evStep = {
        cap.ae.evStep.numerator, cap.ae.evStep.denominator
    };
    cm.update(ANDROID_CONTROL_AE_COMPENSATION_STEP, &evStep, 1);
    const int32_t evRange[] = { cap.ae.evMinSteps, cap.ae.evMaxSteps };
    cm.update(ANDROID_CONTROL_AE_COMPENSATION_RANGE, evRange, NELEM(evRange));

    static const uint8_t controlAeAvailableAntibandingModes[] = {
            ANDROID_CONTROL_AE_ANTIBANDING_MODE_OFF
    };
    cm.update(ANDROID_CONTROL_AE_AVAILABLE_ANTIBANDING_MODES, controlAeAvailableAntibandingModes, NELEM(controlAeAvailableAntibandingModes));

    std::vector<uint8_t> awbModes;
    for (AwbMode m : cap.awb.modes) awbModes.push_back((uint8_t)m);
    cm.update(ANDROID_CONTROL_AWB_AVAILABLE_MODES, awbModes.data(), awbModes.size());

    std::vector<uint8_t> afModes;
    for (AfMode m : cap.af.modes) afModes.push_back((uint8_t)m);
    cm.update(ANDROID_CONTROL_AF_AVAILABLE_MODES, afModes.data(), afModes.size());

    static const uint8_t controlAvailableVideoStabilizationModes[] = {
            ANDROID_CONTROL_VIDEO_STABILIZATION_MODE_OFF
    };
    cm.update(ANDROID_CONTROL_AVAILABLE_VIDEO_STABILIZATION_MODES, controlAvailableVideoStabilizationModes, NELEM(controlAvailableVideoStabilizationModes));

    /* High-level CONTROL_MODE values. AUTO drives the per-3A-mode
     * pipeline (AE / AWB / AF mode each consulted independently); OFF
     * gives the framework full manual control of the 3A pipeline.
     * USE_SCENE_MODE is advertised since some apps gate on it even
     * when the only available scene is DISABLED. */
    static const uint8_t controlAvailableModes[] = {
        ANDROID_CONTROL_MODE_OFF,
        ANDROID_CONTROL_MODE_AUTO,
        ANDROID_CONTROL_MODE_USE_SCENE_MODE,
    };
    cm.update(ANDROID_CONTROL_AVAILABLE_MODES,
              controlAvailableModes, NELEM(controlAvailableModes));
}

/* Per-pipeline-stage AVAILABLE_*_MODES arrays. The framework /
 * camera apps query these to know which mode values can legally
 * appear in a request; an absent key returns null on the app side
 * and apps that do `.length` on it crash with NullPointerException
 * (caught Open Camera 1.55 doing this on
 * STATISTICS_INFO_AVAILABLE_FACE_DETECT_MODES — line 2804 of its
 * CameraController2.java). All advertise OFF only since we don't
 * yet implement per-stage shader work for any of them; tonemap
 * advertises FAST as well because TONEMAP_MODE_OFF requires manual
 * tonemap support which we don't claim. */
void writeStageAvailableModes(CameraMetadata &cm) {
    static const uint8_t edgeAvailableModes[] = {
        ANDROID_EDGE_MODE_OFF,
    };
    cm.update(ANDROID_EDGE_AVAILABLE_EDGE_MODES,
              edgeAvailableModes, NELEM(edgeAvailableModes));

    static const uint8_t hotPixelAvailableModes[] = {
        ANDROID_HOT_PIXEL_MODE_OFF,
    };
    cm.update(ANDROID_HOT_PIXEL_AVAILABLE_HOT_PIXEL_MODES,
              hotPixelAvailableModes, NELEM(hotPixelAvailableModes));

    static const uint8_t noiseReductionAvailableModes[] = {
        ANDROID_NOISE_REDUCTION_MODE_OFF,
    };
    cm.update(ANDROID_NOISE_REDUCTION_AVAILABLE_NOISE_REDUCTION_MODES,
              noiseReductionAvailableModes, NELEM(noiseReductionAvailableModes));

    static const uint8_t shadingAvailableModes[] = {
        ANDROID_SHADING_MODE_OFF,
    };
    cm.update(ANDROID_SHADING_AVAILABLE_MODES,
              shadingAvailableModes, NELEM(shadingAvailableModes));

    static const uint8_t tonemapAvailableModes[] = {
        ANDROID_TONEMAP_MODE_FAST,
    };
    cm.update(ANDROID_TONEMAP_AVAILABLE_TONE_MAP_MODES,
              tonemapAvailableModes, NELEM(tonemapAvailableModes));

    static const uint8_t colorCorrectionAberrationModes[] = {
        ANDROID_COLOR_CORRECTION_ABERRATION_MODE_OFF,
    };
    cm.update(ANDROID_COLOR_CORRECTION_AVAILABLE_ABERRATION_MODES,
              colorCorrectionAberrationModes,
              NELEM(colorCorrectionAberrationModes));

    static const uint8_t lensOpticalStabModes[] = {
        ANDROID_LENS_OPTICAL_STABILIZATION_MODE_OFF,
    };
    cm.update(ANDROID_LENS_INFO_AVAILABLE_OPTICAL_STABILIZATION,
              lensOpticalStabModes, NELEM(lensOpticalStabModes));

    static const int32_t sensorAvailableTestPatternModes[] = {
        ANDROID_SENSOR_TEST_PATTERN_MODE_OFF,
    };
    cm.update(ANDROID_SENSOR_AVAILABLE_TEST_PATTERN_MODES,
              sensorAvailableTestPatternModes,
              NELEM(sensorAvailableTestPatternModes));
}

/* Lens calibration scalars + per-sensor sensor-info bits the
 * framework expects on a non-LEGACY HAL. Hyperfocal is left at 0
 * (== "unknown / treat as infinity") since we have no per-module
 * calibration; FOCUS_DISTANCE_CALIBRATION is UNCALIBRATED for the
 * same reason. APERTURE / FILTER_DENSITIES are single-element
 * arrays with sensible defaults — apps iterate length so empty
 * arrays would crash. */
void writeLensCalibration(CameraMetadata &cm, const Capability &cap) {
    static const float lensInfoHyperfocalDistance = 0.0f;
    cm.update(ANDROID_LENS_INFO_HYPERFOCAL_DISTANCE,
              &lensInfoHyperfocalDistance, 1);

    static const uint8_t lensInfoFocusDistanceCalibration =
        ANDROID_LENS_INFO_FOCUS_DISTANCE_CALIBRATION_UNCALIBRATED;
    cm.update(ANDROID_LENS_INFO_FOCUS_DISTANCE_CALIBRATION,
              &lensInfoFocusDistanceCalibration, 1);

    const float lensInfoAvailableApertures[] = { cap.lens.aperture };
    cm.update(ANDROID_LENS_INFO_AVAILABLE_APERTURES,
              lensInfoAvailableApertures, NELEM(lensInfoAvailableApertures));

    static const float lensInfoAvailableFilterDensities[] = { 0.0f };
    cm.update(ANDROID_LENS_INFO_AVAILABLE_FILTER_DENSITIES,
              lensInfoAvailableFilterDensities,
              NELEM(lensInfoAvailableFilterDensities));

    static const uint8_t sensorInfoTimestampSource =
        ANDROID_SENSOR_INFO_TIMESTAMP_SOURCE_UNKNOWN;
    cm.update(ANDROID_SENSOR_INFO_TIMESTAMP_SOURCE,
              &sensorInfoTimestampSource, 1);

    const uint8_t cfa = (uint8_t)cap.sensor.cfa;
    cm.update(ANDROID_SENSOR_INFO_COLOR_FILTER_ARRANGEMENT, &cfa, 1);
}

/* HAL capability level + request-pipeline shape; the depth is the
 * core's (Capability::pipelineMaxDepth). */
void writeHalInfo(CameraMetadata &cm, const Capability &cap) {
    const uint8_t infoSupportedHardwareLevel = ANDROID_INFO_SUPPORTED_HARDWARE_LEVEL_LIMITED;
    cm.update(ANDROID_INFO_SUPPORTED_HARDWARE_LEVEL, &infoSupportedHardwareLevel, 1);

    const uint8_t pipelineMaxDepth = (uint8_t)cap.pipelineMaxDepth;
    cm.update(ANDROID_REQUEST_PIPELINE_MAX_DEPTH,   &pipelineMaxDepth,    1);
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
     * and the BLOB encode runs on the JPEG engine concurrently with
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
    const int32_t maxOutputProc     = cap.maxProcessedOutputs;
    const int32_t maxOutputStalling = cap.maxStallingOutputs;
    cm.update(ANDROID_REQUEST_MAX_NUM_OUTPUT_RAW,
              &maxOutputRaw,      1);
    cm.update(ANDROID_REQUEST_MAX_NUM_OUTPUT_PROC,
              &maxOutputProc,     1);
    cm.update(ANDROID_REQUEST_MAX_NUM_OUTPUT_PROC_STALLING,
              &maxOutputStalling, 1);
#else
    const int32_t maxOutputStreams[] = {
        0,                        /* RAW                */
        cap.maxProcessedOutputs,  /* PROCESSED          */
        cap.maxStallingOutputs,   /* PROCESSED_STALLING */
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
        ANDROID_BLACK_LEVEL_LOCK,
        ANDROID_COLOR_CORRECTION_ABERRATION_MODE,
        ANDROID_COLOR_CORRECTION_GAINS,
        ANDROID_COLOR_CORRECTION_MODE,
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
        ANDROID_CONTROL_CAPTURE_INTENT,
        ANDROID_CONTROL_EFFECT_MODE,
        ANDROID_CONTROL_MODE,
        ANDROID_CONTROL_SCENE_MODE,
        ANDROID_CONTROL_VIDEO_STABILIZATION_MODE,
        ANDROID_EDGE_MODE,
        ANDROID_FLASH_MODE,
        ANDROID_HOT_PIXEL_MODE,
        ANDROID_JPEG_ORIENTATION,
        ANDROID_JPEG_QUALITY,
        ANDROID_JPEG_THUMBNAIL_QUALITY,
        ANDROID_JPEG_THUMBNAIL_SIZE,
        ANDROID_LENS_FOCUS_DISTANCE,
        ANDROID_LENS_OPTICAL_STABILIZATION_MODE,
        ANDROID_NOISE_REDUCTION_MODE,
        ANDROID_REQUEST_ID,
        ANDROID_SCALER_CROP_REGION,
        ANDROID_SENSOR_EXPOSURE_TIME,
        ANDROID_SENSOR_SENSITIVITY,
        ANDROID_SENSOR_TEST_PATTERN_MODE,
        ANDROID_SHADING_MODE,
        ANDROID_STATISTICS_FACE_DETECT_MODE,
        ANDROID_STATISTICS_HOT_PIXEL_MAP_MODE,
        ANDROID_STATISTICS_LENS_SHADING_MAP_MODE,
        ANDROID_TONEMAP_MODE,
    };
    cm.update(ANDROID_REQUEST_AVAILABLE_REQUEST_KEYS,
              requestKeys, NELEM(requestKeys));

    /* Result keys = round-tripped request keys (any key the app set
     * lands back unchanged in cm) plus result-only keys the
     * ResultMetadataBuilder writes per frame. */
    static const int32_t resultKeys[] = {
        /* Round-tripped request side */
        ANDROID_BLACK_LEVEL_LOCK,
        ANDROID_COLOR_CORRECTION_ABERRATION_MODE,
        ANDROID_COLOR_CORRECTION_GAINS,
        ANDROID_COLOR_CORRECTION_MODE,
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
        ANDROID_CONTROL_CAPTURE_INTENT,
        ANDROID_CONTROL_EFFECT_MODE,
        ANDROID_CONTROL_MODE,
        ANDROID_CONTROL_SCENE_MODE,
        ANDROID_CONTROL_VIDEO_STABILIZATION_MODE,
        ANDROID_EDGE_MODE,
        ANDROID_FLASH_MODE,
        ANDROID_HOT_PIXEL_MODE,
        ANDROID_JPEG_ORIENTATION,
        ANDROID_JPEG_QUALITY,
        ANDROID_JPEG_THUMBNAIL_QUALITY,
        ANDROID_JPEG_THUMBNAIL_SIZE,
        ANDROID_LENS_OPTICAL_STABILIZATION_MODE,
        ANDROID_NOISE_REDUCTION_MODE,
        ANDROID_REQUEST_ID,
        ANDROID_SCALER_CROP_REGION,
        ANDROID_SENSOR_TEST_PATTERN_MODE,
        ANDROID_SHADING_MODE,
        ANDROID_STATISTICS_FACE_DETECT_MODE,
        ANDROID_STATISTICS_HOT_PIXEL_MAP_MODE,
        ANDROID_STATISTICS_LENS_SHADING_MAP_MODE,
        ANDROID_TONEMAP_MODE,
        /* Result-only — written per-frame by ResultMetadataBuilder */
        ANDROID_CONTROL_AE_STATE,
        ANDROID_CONTROL_AF_STATE,
        ANDROID_CONTROL_AWB_STATE,
        ANDROID_FLASH_STATE,
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
        ANDROID_COLOR_CORRECTION_AVAILABLE_ABERRATION_MODES,
        ANDROID_CONTROL_AE_AVAILABLE_ANTIBANDING_MODES,
        ANDROID_CONTROL_AE_AVAILABLE_MODES,
        ANDROID_CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES,
        ANDROID_CONTROL_AE_COMPENSATION_RANGE,
        ANDROID_CONTROL_AE_COMPENSATION_STEP,
        ANDROID_CONTROL_AF_AVAILABLE_MODES,
        ANDROID_CONTROL_AVAILABLE_EFFECTS,
        ANDROID_CONTROL_AVAILABLE_MODES,
        ANDROID_CONTROL_AVAILABLE_SCENE_MODES,
        ANDROID_CONTROL_AVAILABLE_VIDEO_STABILIZATION_MODES,
        ANDROID_CONTROL_AWB_AVAILABLE_MODES,
        ANDROID_CONTROL_MAX_REGIONS,
        ANDROID_EDGE_AVAILABLE_EDGE_MODES,
        ANDROID_FLASH_INFO_AVAILABLE,
        ANDROID_HOT_PIXEL_AVAILABLE_HOT_PIXEL_MODES,
        ANDROID_INFO_SUPPORTED_HARDWARE_LEVEL,
        ANDROID_JPEG_AVAILABLE_THUMBNAIL_SIZES,
        ANDROID_JPEG_MAX_SIZE,
        ANDROID_LENS_FACING,
        ANDROID_LENS_INFO_AVAILABLE_APERTURES,
        ANDROID_LENS_INFO_AVAILABLE_FILTER_DENSITIES,
        ANDROID_LENS_INFO_AVAILABLE_FOCAL_LENGTHS,
        ANDROID_LENS_INFO_AVAILABLE_OPTICAL_STABILIZATION,
        ANDROID_LENS_INFO_FOCUS_DISTANCE_CALIBRATION,
        ANDROID_LENS_INFO_HYPERFOCAL_DISTANCE,
        ANDROID_LENS_INFO_MINIMUM_FOCUS_DISTANCE,
        ANDROID_NOISE_REDUCTION_AVAILABLE_NOISE_REDUCTION_MODES,
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
        ANDROID_SENSOR_AVAILABLE_TEST_PATTERN_MODES,
        ANDROID_SENSOR_INFO_ACTIVE_ARRAY_SIZE,
        ANDROID_SENSOR_INFO_COLOR_FILTER_ARRANGEMENT,
        ANDROID_SENSOR_INFO_EXPOSURE_TIME_RANGE,
        ANDROID_SENSOR_INFO_MAX_FRAME_DURATION,
        ANDROID_SENSOR_INFO_PHYSICAL_SIZE,
        ANDROID_SENSOR_INFO_PIXEL_ARRAY_SIZE,
        ANDROID_SENSOR_INFO_SENSITIVITY_RANGE,
        ANDROID_SENSOR_INFO_TIMESTAMP_SOURCE,
        ANDROID_SENSOR_MAX_ANALOG_SENSITIVITY,
        ANDROID_SENSOR_ORIENTATION,
        ANDROID_SHADING_AVAILABLE_MODES,
        ANDROID_STATISTICS_INFO_AVAILABLE_FACE_DETECT_MODES,
        ANDROID_STATISTICS_INFO_AVAILABLE_HOT_PIXEL_MAP_MODES,
        ANDROID_STATISTICS_INFO_AVAILABLE_LENS_SHADING_MAP_MODES,
        ANDROID_STATISTICS_INFO_MAX_FACE_COUNT,
        ANDROID_SYNC_MAX_LATENCY,
        ANDROID_TONEMAP_AVAILABLE_TONE_MAP_MODES,
    };
    cm.update(ANDROID_REQUEST_AVAILABLE_CHARACTERISTICS_KEYS,
              characteristicsKeys, NELEM(characteristicsKeys));
}

} /* namespace */

camera_metadata_t *CameraStaticMetadata::build(const Capability &cap) {
    CameraMetadata cm;
    writeSensorInfo         (cm, cap);
    writeScalerConfigs      (cm, cap);
    writeJpegInfo           (cm, cap);
    writeSensorRanges       (cm, cap);
    writeAvailableFpsRanges (cm, cap);
    writeControlInfo        (cm, cap);
    writeStageAvailableModes(cm);
    writeLensCalibration    (cm, cap);
    writeHalInfo            (cm, cap);
    writeAvailableKeys      (cm);
    return cm.release();
}

}; /* namespace android */
