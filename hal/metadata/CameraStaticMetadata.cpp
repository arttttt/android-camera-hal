#define LOG_TAG "Cam-StaticMeta"

#include "CameraStaticMetadata.h"

#include <stdint.h>
#include <string.h>
#include <sys/user.h>
#include <linux/videodev2.h>

#include <hardware/camera3.h>
#include <hardware/camera_common.h>
#include <camera/CameraMetadata.h>
#include <utils/misc.h>

#include "V4l2Device.h"
#include "OutputResolutionCap.h"
#include "sensor/SensorConfig.h"
#include "sensor/SensorTuning.h"

namespace android {

namespace {

/* Pipeline depth advertised to the framework so it can size its
 * own backpressure / scheduler windows. After Tier 3 PR 4 we run a
 * fence-fd ring of depth 4 between RequestThread and PipelineThread
 * (PipelineContext.SLOT_COUNT in the Vulkan ISP, GPU-submit ring of
 * the same size). `kPartialResultCount` lives in
 * CameraStaticMetadata.h so producers (IPA tick, result-dispatch
 * stage) can reference the same advertised value. */
constexpr uint8_t  kPipelineMaxDepth   = 4;

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

    /* From per-module tuning (`module.sensor_orientation_degrees`).
     * Camera2 spec mandates one of {0, 90, 180, 270}; the static
     * characteristics version of the same value Camera::cameraInfo
     * already reports as camera_info::orientation. */
    const int32_t sensorOrientation = tuning ? tuning->module().sensorOrientationDegrees : 0;
    cm.update(ANDROID_SENSOR_ORIENTATION, &sensorOrientation, 1);
}

/* Resolution × format tables + per-mode durations. This is the bulk of
 * the characteristics — every supported (format, width, height) pair
 * needs to be enumerated here so CameraX / Camera2 can pick a stream.
 *
 * Filtered against `OutputResolutionCap` — V4L2 enumerates every native
 * sensor mode (incl. 8 MP / 5 MP for IMX179 / OV5693) but we advertise
 * only what the SW ISP can sustain at frame rate, so apps' photo /
 * video resolution pickers stay honest about what the pipeline
 * actually delivers fast. */
void writeScalerConfigs(CameraMetadata &cm, V4l2Device *dev) {
    Vector<Resolution> resolutions;
    {
        const auto &all = dev->availableResolutions();
        for (size_t i = 0; i < all.size(); ++i) {
            const auto &r = all[i];
            if (OutputResolutionCap::accepts((int32_t)r.width, (int32_t)r.height))
                resolutions.add(r);
        }
    }
    const auto &previewResolutions = resolutions;

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

/* Declared sensor operating ranges — exposure time + ISO sensitivity,
 * sourced from the V4L2 driver's queryctrl. No fallback constants:
 * if the driver can't answer the sensor ranges then nothing else in
 * this HAL works, and a wrong value advertised here would just lie to
 * apps about what the slider should offer. Both keys get skipped and
 * the framework will reject the HAL — that's the right outcome,
 * surface the breakage instead of papering over it.
 *
 * Gain↔ISO conversion goes through SensorConfig::gainToIso so the
 * sensor's gainUnit (Q8 today, could differ on a future module) and
 * the Camera2 ISO-at-unity anchor live in one place. */
void writeSensorRanges(CameraMetadata &cm, V4l2Device *dev,
                        const SensorConfig &cfg) {
    int32_t expMinUs = 0, expMaxUs = 0, expDefUs = 0;
    if (!dev->queryControl(V4L2_CID_EXPOSURE, &expMinUs, &expMaxUs, &expDefUs)) {
        ALOGE("writeSensorRanges: V4L2_CID_EXPOSURE query failed — "
              "EXPOSURE_TIME_RANGE will be missing from characteristics");
        return;
    }

    int32_t gMin = 0, gMax = 0, gDef = 0;
    if (!dev->queryControl(V4L2_CID_GAIN, &gMin, &gMax, &gDef)) {
        ALOGE("writeSensorRanges: V4L2_CID_GAIN query failed — "
              "SENSITIVITY_RANGE will be missing from characteristics");
        return;
    }

    /* V4L2_CID_EXPOSURE returns microseconds; metadata wants ns. */
    const int64_t sensorExposureTimeRange[] = {
        (int64_t)expMinUs * 1000LL,
        (int64_t)expMaxUs * 1000LL,
    };
    cm.update(ANDROID_SENSOR_INFO_EXPOSURE_TIME_RANGE,
              sensorExposureTimeRange, NELEM(sensorExposureTimeRange));

    /* V4L2_CID_GAIN on these sensors is pure analog gain — advertise
     * the driver-reported max as MAX_ANALOG too. */
    int32_t isoMin = cfg.gainToIso(gMin);
    int32_t isoMax = cfg.gainToIso(gMax);
    if (isoMin < 1) isoMin = 1;
    const int32_t sensorSensitivityRange[] = { isoMin, isoMax };
    cm.update(ANDROID_SENSOR_INFO_SENSITIVITY_RANGE,
              sensorSensitivityRange, NELEM(sensorSensitivityRange));

    const int32_t sensorMaxAnalogSensitivity = isoMax;
    cm.update(ANDROID_SENSOR_MAX_ANALOG_SENSITIVITY,
              &sensorMaxAnalogSensitivity, 1);
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
void writeLensCalibration(CameraMetadata &cm, const SensorTuning *tuning) {
    static const float lensInfoHyperfocalDistance = 0.0f;
    cm.update(ANDROID_LENS_INFO_HYPERFOCAL_DISTANCE,
              &lensInfoHyperfocalDistance, 1);

    static const uint8_t lensInfoFocusDistanceCalibration =
        ANDROID_LENS_INFO_FOCUS_DISTANCE_CALIBRATION_UNCALIBRATED;
    cm.update(ANDROID_LENS_INFO_FOCUS_DISTANCE_CALIBRATION,
              &lensInfoFocusDistanceCalibration, 1);

    static const float lensInfoAvailableApertures[] = { 2.0f };
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

    /* Map the tuning's bayer_pattern string to the camera2 enum. */
    uint8_t cfa = ANDROID_SENSOR_INFO_COLOR_FILTER_ARRANGEMENT_RGGB;
    if (tuning && tuning->isLoaded()) {
        const char *p = tuning->bayerPattern();
        if      (strcmp(p, "RGGB") == 0) cfa = ANDROID_SENSOR_INFO_COLOR_FILTER_ARRANGEMENT_RGGB;
        else if (strcmp(p, "GRBG") == 0) cfa = ANDROID_SENSOR_INFO_COLOR_FILTER_ARRANGEMENT_GRBG;
        else if (strcmp(p, "GBRG") == 0) cfa = ANDROID_SENSOR_INFO_COLOR_FILTER_ARRANGEMENT_GBRG;
        else if (strcmp(p, "BGGR") == 0) cfa = ANDROID_SENSOR_INFO_COLOR_FILTER_ARRANGEMENT_BGGR;
    }
    cm.update(ANDROID_SENSOR_INFO_COLOR_FILTER_ARRANGEMENT, &cfa, 1);
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
        ANDROID_CONTROL_AWB_REGIONS,
        ANDROID_CONTROL_CAPTURE_INTENT,
        ANDROID_CONTROL_EFFECT_MODE,
        ANDROID_CONTROL_MODE,
        ANDROID_CONTROL_SCENE_MODE,
        ANDROID_CONTROL_VIDEO_STABILIZATION_MODE,
        ANDROID_EDGE_MODE,
        ANDROID_HOT_PIXEL_MODE,
        ANDROID_JPEG_GPS_COORDINATES,
        ANDROID_JPEG_GPS_PROCESSING_METHOD,
        ANDROID_JPEG_GPS_TIMESTAMP,
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
        ANDROID_STATISTICS_HISTOGRAM_MODE,
        ANDROID_STATISTICS_HOT_PIXEL_MAP_MODE,
        ANDROID_STATISTICS_LENS_SHADING_MAP_MODE,
        ANDROID_STATISTICS_SHARPNESS_MAP_MODE,
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
        ANDROID_CONTROL_AWB_REGIONS,
        ANDROID_CONTROL_CAPTURE_INTENT,
        ANDROID_CONTROL_EFFECT_MODE,
        ANDROID_CONTROL_MODE,
        ANDROID_CONTROL_SCENE_MODE,
        ANDROID_CONTROL_VIDEO_STABILIZATION_MODE,
        ANDROID_EDGE_MODE,
        ANDROID_HOT_PIXEL_MODE,
        ANDROID_JPEG_GPS_COORDINATES,
        ANDROID_JPEG_GPS_PROCESSING_METHOD,
        ANDROID_JPEG_GPS_TIMESTAMP,
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
        ANDROID_STATISTICS_HISTOGRAM_MODE,
        ANDROID_STATISTICS_HOT_PIXEL_MAP_MODE,
        ANDROID_STATISTICS_LENS_SHADING_MAP_MODE,
        ANDROID_STATISTICS_SHARPNESS_MAP_MODE,
        ANDROID_TONEMAP_MODE,
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

camera_metadata_t *CameraStaticMetadata::build(V4l2Device *dev, int facing,
                                                const SensorTuning *tuning,
                                                const SensorConfig &sensorCfg,
                                                size_t *jpegBufferSize) {
    CameraMetadata cm;
    writeSensorInfo         (cm, dev, tuning, facing);
    writeScalerConfigs      (cm, dev);
    writeJpegInfo           (cm, dev, jpegBufferSize);
    writeSensorRanges       (cm, dev, sensorCfg);
    writeAvailableFpsRanges (cm, dev);
    writeControlInfo        (cm, facing);
    writeStageAvailableModes(cm);
    writeLensCalibration    (cm, tuning);
    writeHalInfo            (cm);
    writeAvailableKeys      (cm);
    return cm.release();
}

}; /* namespace android */
