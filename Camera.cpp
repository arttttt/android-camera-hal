/*
 * Copyright (C) 2015-2016 Antmicro
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define LOG_TAG "Cam-Camera"
#define LOG_NDEBUG NDEBUG

#include <hardware/camera3.h>
#include <camera/CameraMetadata.h>
#include <utils/misc.h>
#include <utils/Log.h>
#include <hardware/gralloc.h>
#include <ui/Rect.h>
#include <ui/GraphicBuffer.h>
#include <ui/GraphicBufferMapper.h>
#include <ui/Fence.h>
#include <assert.h>
#include <linux/videodev2.h>
#include <libyuv/scale_argb.h>
#include <cutils/properties.h>

/* Tegra camera CIDs — not in standard V4L2 headers */
#ifndef V4L2_CID_FRAME_LENGTH
#define V4L2_CID_FRAME_LENGTH   (V4L2_CTRL_CLASS_CAMERA | 0x2000)
#endif

#include "DbgUtils.h"
#include "Camera.h"
#include "ImageConverter.h"
#include "CpuIspPipeline.h"
#include "VulkanIspPipeline.h"
#include "GlesIspPipeline.h"
#include "IspCalibration.h"

extern camera_module_t HAL_MODULE_INFO_SYM;

namespace android {
/**
 * \class Camera
 *
 * Android's Camera 3 device implementation.
 *
 * Declaration of camera capabilities, frame request handling, etc. This code
 * is what Android framework talks to.
 */

Camera::Camera(const char *devNode, int facing)
    : mStaticCharacteristics(NULL)
    , mCallbackOps(NULL)
    , mFacing(facing)
    , mJpegBufferSize(0)
    , mRgbaTemp(NULL)
    , mRgbaTempSize(0)
    , mV4l2Width(0)
    , mV4l2Height(0)
    , mSoftIspEnabled(true)
    , mFocusPosition(140)
    , mAfSweepActive(false)
    , mAfSweepPos(0)
    , mAfSweepStep(0)
    , mAfSweepEnd(0)
    , mAfSweepBestPos(140)
    , mAfSweepBestScore(0)
    , mAfFinePass(false)
    , mAfSettleFrames(0) {
    DBGUTILS_AUTOLOGCALL(__func__);
    for(size_t i = 0; i < NELEM(mDefaultRequestSettings); i++) {
        mDefaultRequestSettings[i] = NULL;
    }

    common.tag      = HARDWARE_DEVICE_TAG;
    common.version  = CAMERA_DEVICE_API_VERSION_3_0;
    common.module   = &HAL_MODULE_INFO_SYM.common;
    common.close    = Camera::sClose;
    ops             = &sOps;
    priv            = NULL;

    mValid = true;
    mIsp = NULL;
    mDev = new V4l2Device(devNode);
    if(!mDev) {
        mValid = false;
    }
}

Camera::~Camera() {
    DBGUTILS_AUTOLOGCALL(__func__);
    gWorkers.stop();
    if (mIsp) { mIsp->destroy(); delete mIsp; }
    mDev->disconnect();
    delete mDev;
    free(mRgbaTemp);
}

status_t Camera::cameraInfo(struct camera_info *info) {
    DBGUTILS_AUTOLOGCALL(__func__);
    Mutex::Autolock lock(mMutex);
    info->facing = mFacing;
    info->orientation = (mFacing == CAMERA_FACING_FRONT) ? 270 : 90;
    info->device_version = CAMERA_DEVICE_API_VERSION_3_0;
    info->static_camera_characteristics = staticCharacteristics();

    return NO_ERROR;
}

int Camera::openDevice(hw_device_t **device) {
    DBGUTILS_AUTOLOGCALL(__func__);
    Mutex::Autolock lock(mMutex);
    mDev->connect();
    *device = &common;

    /* Open focuser for back camera */
    if (mFacing == CAMERA_FACING_BACK)
        mDev->openFocuser("/dev/v4l-subdev0");

    gWorkers.start();

    return NO_ERROR;
}

int Camera::closeDevice() {
    DBGUTILS_AUTOLOGCALL(__func__);
    Mutex::Autolock lock(mMutex);

    gWorkers.stop();
    mDev->disconnect();

    return NO_ERROR;
}

camera_metadata_t *Camera::staticCharacteristics() {
    if(mStaticCharacteristics)
        return mStaticCharacteristics;

    CameraMetadata cm;

    auto &resolutions = mDev->availableResolutions();
    auto &previewResolutions = resolutions;
    auto sensorRes = mDev->sensorResolution();

    /***********************************\
    |* START OF CAMERA CHARACTERISTICS *|
    \***********************************/

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
    if (mFacing == CAMERA_FACING_BACK) {
        static const float lensInfoMinFocusDistance = 10.0f; /* 1/10m = 10cm macro */
        cm.update(ANDROID_LENS_INFO_MINIMUM_FOCUS_DISTANCE, &lensInfoMinFocusDistance, 1);
    } else {
        static const float lensInfoMinFocusDistance = 0.0f; /* fixed focus */
        cm.update(ANDROID_LENS_INFO_MINIMUM_FOCUS_DISTANCE, &lensInfoMinFocusDistance, 1);
    }

    const uint8_t lensFacing = (mFacing == CAMERA_FACING_FRONT)
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

    static const int32_t scalerAvailableFormats[] = {
        HAL_PIXEL_FORMAT_RGBA_8888,
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
        scalerAvailableStreamConfigurations[i4 + 0] = HAL_PIXEL_FORMAT_BLOB;
        scalerAvailableStreamConfigurations[i4 + 1] = (int32_t)resolutions[resId].width;
        scalerAvailableStreamConfigurations[i4 + 2] = (int32_t)resolutions[resId].height;
        scalerAvailableStreamConfigurations[i4 + 3] = ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT;

        scalerAvailableMinFrameDurations[i4 + 0] = HAL_PIXEL_FORMAT_BLOB;
        scalerAvailableMinFrameDurations[i4 + 1] = (int32_t)resolutions[resId].width;
        scalerAvailableMinFrameDurations[i4 + 2] = (int32_t)resolutions[resId].height;
        scalerAvailableMinFrameDurations[i4 + 3] = 1000000000 / 60; /* TODO: read from the device */

        scalerAvailableJpegSizes[i2 + 0] = (int32_t)resolutions[resId].width;
        scalerAvailableJpegSizes[i2 + 1] = (int32_t)resolutions[resId].height;

        scalerAvailableJpegMinDurations[i1] = 1000000000 / 60; /* TODO: read from the device */

        i4 += 4;
        i2 += 2;
        i1 += 1;
    }
    i2 = 0;
    i1 = 0;
    /* Preview stream configurations */
    for(size_t resId = 0; resId < previewResolutions.size(); ++resId) {
        for(size_t fmtId = 0; fmtId < NELEM(scalerAvailableFormats) - 1; ++fmtId) {
            scalerAvailableStreamConfigurations[i4 + 0] = scalerAvailableFormats[fmtId];
            scalerAvailableStreamConfigurations[i4 + 1] = (int32_t)previewResolutions[resId].width;
            scalerAvailableStreamConfigurations[i4 + 2] = (int32_t)previewResolutions[resId].height;
            scalerAvailableStreamConfigurations[i4 + 3] = ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS_OUTPUT;

            scalerAvailableMinFrameDurations[i4 + 0] = scalerAvailableFormats[fmtId];
            scalerAvailableMinFrameDurations[i4 + 1] = (int32_t)previewResolutions[resId].width;
            scalerAvailableMinFrameDurations[i4 + 2] = (int32_t)previewResolutions[resId].height;
            scalerAvailableMinFrameDurations[i4 + 3] = 1000000000 / 60; /* TODO: read from the device */

            i4 += 4;
        }
        scalerAvailableProcessedSizes[i2 + 0] = (int32_t)previewResolutions[resId].width;
        scalerAvailableProcessedSizes[i2 + 1] = (int32_t)previewResolutions[resId].height;

        scalerAvailableProcessedMinDurations[i1] = 1000000000 / 60; /* TODO: read from the device */

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

    /* ~8.25 bit/px (https://en.wikipedia.org/wiki/JPEG#Sample_photographs) */
    /* Use 9 bit/px, add buffer info struct size, round up to page size */
    mJpegBufferSize = sensorRes.width * sensorRes.height * 9 + sizeof(camera3_jpeg_blob);
    mJpegBufferSize = (mJpegBufferSize + PAGE_SIZE - 1u) & ~(PAGE_SIZE - 1u);
    const int32_t jpegMaxSize = (int32_t)mJpegBufferSize;
    cm.update(ANDROID_JPEG_MAX_SIZE, &jpegMaxSize, 1);

    static const int32_t jpegAvailableThumbnailSizes[] = {
        0, 0,
        320, 240
    };
    cm.update(ANDROID_JPEG_AVAILABLE_THUMBNAIL_SIZES, jpegAvailableThumbnailSizes, NELEM(jpegAvailableThumbnailSizes));

    const int32_t sensorOrientation = (mFacing == CAMERA_FACING_FRONT) ? 270 : 90;
    cm.update(ANDROID_SENSOR_ORIENTATION, &sensorOrientation, 1);

    static const uint8_t flashInfoAvailable = ANDROID_FLASH_INFO_AVAILABLE_FALSE;
    cm.update(ANDROID_FLASH_INFO_AVAILABLE, &flashInfoAvailable, 1);

    static const float scalerAvailableMaxDigitalZoom = 4;
    cm.update(ANDROID_SCALER_AVAILABLE_MAX_DIGITAL_ZOOM, &scalerAvailableMaxDigitalZoom, 1);

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

    /* Exposure time: 0.1ms to 200ms */
    static const int64_t sensorExposureTimeRange[] = { 100000LL, 200000000LL };
    cm.update(ANDROID_SENSOR_INFO_EXPOSURE_TIME_RANGE, sensorExposureTimeRange, NELEM(sensorExposureTimeRange));

    /* ISO sensitivity: 100 to 3200 */
    static const int32_t sensorSensitivityRange[] = { 100, 3200 };
    cm.update(ANDROID_SENSOR_INFO_SENSITIVITY_RANGE, sensorSensitivityRange, NELEM(sensorSensitivityRange));

    static const int32_t sensorMaxAnalogSensitivity = 1600;
    cm.update(ANDROID_SENSOR_MAX_ANALOG_SENSITIVITY, &sensorMaxAnalogSensitivity, 1);

    static const uint8_t controlAwbAvailableModes[] = {
            ANDROID_CONTROL_AWB_MODE_AUTO,
            ANDROID_CONTROL_AWB_MODE_OFF
    };
    cm.update(ANDROID_CONTROL_AWB_AVAILABLE_MODES, controlAwbAvailableModes, NELEM(controlAwbAvailableModes));

    if (mFacing == CAMERA_FACING_BACK) {
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

    const uint8_t infoSupportedHardwareLevel = ANDROID_INFO_SUPPORTED_HARDWARE_LEVEL_LIMITED;
    cm.update(ANDROID_INFO_SUPPORTED_HARDWARE_LEVEL, &infoSupportedHardwareLevel, 1);

    /***********************************\
    |*  END OF CAMERA CHARACTERISTICS  *|
    \***********************************/

    mStaticCharacteristics = cm.release();
    return mStaticCharacteristics;
}

int Camera::initialize(const camera3_callback_ops_t *callbackOps) {
    DBGUTILS_AUTOLOGCALL(__func__);
    Mutex::Autolock lock(mMutex);

    mCallbackOps = callbackOps;
    return NO_ERROR;
}

const camera_metadata_t * Camera::constructDefaultRequestSettings(int type) {
    DBGUTILS_AUTOLOGCALL(__func__);
    Mutex::Autolock lock(mMutex);
    /* TODO: validate type */

    if(mDefaultRequestSettings[type]) {
        return mDefaultRequestSettings[type];
    }

    CameraMetadata cm;

    static const int32_t requestId = 0;
    cm.update(ANDROID_REQUEST_ID, &requestId, 1);

    static const float lensFocusDistance = 0.0f;
    cm.update(ANDROID_LENS_FOCUS_DISTANCE, &lensFocusDistance, 1);

    auto sensorSize = mDev->sensorResolution();
    const int32_t scalerCropRegion[] = {
        0,                          0,
        (int32_t)sensorSize.width,  (int32_t)sensorSize.height
    };
    cm.update(ANDROID_SCALER_CROP_REGION, scalerCropRegion, NELEM(scalerCropRegion));

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

    /** android.stats */

    static const uint8_t statisticsFaceDetectMode = ANDROID_STATISTICS_FACE_DETECT_MODE_OFF;
    cm.update(ANDROID_STATISTICS_FACE_DETECT_MODE, &statisticsFaceDetectMode, 1);

    static const uint8_t statisticsHistogramMode = ANDROID_STATISTICS_HISTOGRAM_MODE_OFF;
    cm.update(ANDROID_STATISTICS_HISTOGRAM_MODE, &statisticsHistogramMode, 1);

    static const uint8_t statisticsSharpnessMapMode = ANDROID_STATISTICS_SHARPNESS_MAP_MODE_OFF;
    cm.update(ANDROID_STATISTICS_SHARPNESS_MAP_MODE, &statisticsSharpnessMapMode, 1);

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

    static const int32_t controlAeRegions[] = {
        0,                          0,
        (int32_t)sensorSize.width,  (int32_t)sensorSize.height,
        1000
    };
    cm.update(ANDROID_CONTROL_AE_REGIONS, controlAeRegions, NELEM(controlAeRegions));
    cm.update(ANDROID_CONTROL_AWB_REGIONS, controlAeRegions, NELEM(controlAeRegions));
    cm.update(ANDROID_CONTROL_AF_REGIONS, controlAeRegions, NELEM(controlAeRegions));

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

    uint8_t controlAfMode = (mFacing == CAMERA_FACING_BACK) ?
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

    mDefaultRequestSettings[type] = cm.release();
    return mDefaultRequestSettings[type];
}

int Camera::configureStreams(camera3_stream_configuration_t *streamList) {
    DBGUTILS_AUTOLOGCALL(__func__);
    Mutex::Autolock lock(mMutex);

    /* TODO: sanity checks */

    ALOGV("+-------------------------------------------------------------------------------");
    ALOGV("| STREAMS FROM FRAMEWORK");
    ALOGV("+-------------------------------------------------------------------------------");
    for(size_t i = 0; i < streamList->num_streams; ++i) {
        camera3_stream_t *newStream = streamList->streams[i];
        ALOGV("| p=%p  fmt=0x%.2x  type=%u  usage=0x%.8x  size=%4ux%-4u  buf_no=%u",
              newStream,
              newStream->format,
              newStream->stream_type,
              newStream->usage,
              newStream->width,
              newStream->height,
              newStream->max_buffers);
    }
    ALOGV("+-------------------------------------------------------------------------------");

    camera3_stream_t *inStream = NULL;
    unsigned width = 0;
    unsigned height = 0;
    for(size_t i = 0; i < streamList->num_streams; ++i) {
        camera3_stream_t *newStream = streamList->streams[i];

        if(newStream->stream_type == CAMERA3_STREAM_INPUT || newStream->stream_type == CAMERA3_STREAM_BIDIRECTIONAL) {
            if(inStream) {
                ALOGE("Only one input/bidirectional stream allowed (previous is %p, this %p)", inStream, newStream);
                return BAD_VALUE;
            }
            inStream = newStream;
        }

        if(newStream->format == HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED) {
            newStream->format = HAL_PIXEL_FORMAT_RGBA_8888;
        }

        if(newStream->usage & GRALLOC_USAGE_HW_CAMERA_ZSL) {
            ALOGE("ZSL not supported. Add camera.disable_zsl_mode=1 to build.prop");
            return BAD_VALUE;
        }

        switch(newStream->stream_type) {
            case CAMERA3_STREAM_OUTPUT:         newStream->usage = GRALLOC_USAGE_SW_WRITE_OFTEN;                                break;
            case CAMERA3_STREAM_INPUT:          newStream->usage = GRALLOC_USAGE_SW_READ_OFTEN;                                 break;
            case CAMERA3_STREAM_BIDIRECTIONAL:  newStream->usage = GRALLOC_USAGE_SW_WRITE_OFTEN | GRALLOC_USAGE_SW_READ_OFTEN;  break;
        }
        newStream->max_buffers = 4;

        /*
         * Use the largest NON-BLOB (preview/video) stream for V4L2 resolution.
         * BLOB (JPEG) will be captured at this resolution too — no runtime
         * resolution switch needed for simple capture.
         */
        if(newStream->format != HAL_PIXEL_FORMAT_BLOB) {
            if(newStream->width * newStream->height > width * height) {
                width = newStream->width;
                height = newStream->height;
            }
        }
    }

    /* Fallback: if only BLOB streams, use the largest */
    if (!width || !height) {
        for(size_t i = 0; i < streamList->num_streams; ++i) {
            camera3_stream_t *s = streamList->streams[i];
            if(s->width * s->height > width * height) {
                width = s->width;
                height = s->height;
            }
        }
    }

    /* ISP backend selection — read each configureStreams for runtime toggle */
    char propVal[PROPERTY_VALUE_MAX] = {0};
    property_get("persist.camera.soft_isp", propVal, "1");
    mSoftIspEnabled = (propVal[0] == '1');

    char ispBackend[PROPERTY_VALUE_MAX] = {0};
    property_get("persist.camera.isp_backend", ispBackend, "cpu");

    if (mIsp) { mIsp->destroy(); delete mIsp; mIsp = NULL; }
    if (!strcmp(ispBackend, "vulkan")) {
        mIsp = new VulkanIspPipeline();
    } else if (!strcmp(ispBackend, "gles")) {
        mIsp = new GlesIspPipeline();
    } else {
        mIsp = new CpuIspPipeline();
    }
    if (!mIsp->init()) {
        ALOGE("ISP backend '%s' init failed, falling back to CPU", ispBackend);
        delete mIsp;
        mIsp = new CpuIspPipeline();
        mIsp->init();
    }
    mIsp->setEnabled(mSoftIspEnabled);
    mIsp->setCcm((mFacing == CAMERA_FACING_BACK) ? ccm_imx179 : ccm_ov5693);

    ALOGD("V4L2 target resolution: %ux%u, isp=%s, soft_isp=%d",
          width, height, ispBackend, mSoftIspEnabled);

    if(!mDev->setStreaming(false)) {
        ALOGE("Could not stop streaming");
        return NO_INIT;
    }
    if(!mDev->setResolution(width, height)) {
        ALOGE("Could not set resolution");
        return NO_INIT;
    }

    ALOGV("+-------------------------------------------------------------------------------");
    ALOGV("| STREAMS AFTER CHANGES");
    ALOGV("+-------------------------------------------------------------------------------");
    for(size_t i = 0; i < streamList->num_streams; ++i) {
        const camera3_stream_t *newStream = streamList->streams[i];
        ALOGV("| p=%p  fmt=0x%.2x  type=%u  usage=0x%.8x  size=%4ux%-4u  buf_no=%u",
              newStream,
              newStream->format,
              newStream->stream_type,
              newStream->usage,
              newStream->width,
              newStream->height,
              newStream->max_buffers);
    }
    ALOGV("+-------------------------------------------------------------------------------");

    /* Initialize sensor config */
    mSensorCfg = (mFacing == CAMERA_FACING_BACK) ?
        SensorConfig::imx179() : SensorConfig::ov5693();

    /* Set exposure/gain BEFORE STREAMON — only with soft ISP */
    if (mSoftIspEnabled) {
        mDev->setControl(V4L2_CID_EXPOSURE, mSensorCfg.exposureDefault);
        mDev->setControl(V4L2_CID_GAIN, mSensorCfg.gainDefault);
    }

    if(!mDev->setStreaming(true)) {
        ALOGE("Could not start streaming");
        return NO_INIT;
    }

    /* Update sensor config from driver queries */
    {
        int32_t flMin, flMax, flDef;
        if (mDev->queryControl(V4L2_CID_FRAME_LENGTH, &flMin, &flMax, &flDef)) {
            mSensorCfg.frameLenDefault = flDef;
            mSensorCfg.frameLenMax = (flMax > flDef * 3) ? flDef * 3 : flMax;
            ALOGD("Frame length: def=%d max=%d (driver max=%d)",
                  mSensorCfg.frameLenDefault, mSensorCfg.frameLenMax, flMax);
        }
        int32_t gMin, gMax, gDef;
        if (mDev->queryControl(V4L2_CID_GAIN, &gMin, &gMax, &gDef)) {
            mSensorCfg.gainMax = gMax;
            ALOGD("Gain: min=%d max=%d def=%d", gMin, gMax, gDef);
        }
    }

    /* Cache V4L2 resolution for safe conversion */
    auto v4l2Res = mDev->resolution();
    mV4l2Width = v4l2Res.width;
    mV4l2Height = v4l2Res.height;

    /* Pre-allocate temp RGBA buffer for resolution mismatch */
    size_t needed = mV4l2Width * mV4l2Height * 4;
    if (mRgbaTempSize < needed) {
        free(mRgbaTemp);
        mRgbaTemp = (uint8_t *)malloc(needed);
        mRgbaTempSize = needed;
    }

    return NO_ERROR;
}

int Camera::registerStreamBuffers(const camera3_stream_buffer_set_t *bufferSet) {
    DBGUTILS_AUTOLOGCALL(__func__);
    Mutex::Autolock lock(mMutex);
    ALOGV("+-------------------------------------------------------------------------------");
    ALOGV("| BUFFERS FOR STREAM %p", bufferSet->stream);
    ALOGV("+-------------------------------------------------------------------------------");
    for (size_t i = 0; i < bufferSet->num_buffers; ++i) {
        ALOGV("| p=%p", bufferSet->buffers[i]);
    }
    ALOGV("+-------------------------------------------------------------------------------");

    return OK;
}

int Camera::processCaptureRequest(camera3_capture_request_t *request) {
    assert(request != NULL);
    Mutex::Autolock lock(mMutex);

    BENCHMARK_HERE(120);
    FPSCOUNTER_HERE(120);

    CameraMetadata cm;
    const V4l2Device::VBuffer *frame = NULL;
    auto res = mDev->resolution();
    status_t e;
    Vector<camera3_stream_buffer> buffers;

    auto timestamp = systemTime();

    ALOGV("--- capture request --- f=%-5u in_buf=%p  out_bufs=%p[%u] --- fps %4.1f (avg %4.1f)",
          request->frame_number,
          request->input_buffer,
          request->output_buffers,
          request->num_output_buffers,
          FPSCOUNTER_VALUE(1), FPSCOUNTER_VALUE());

    if(request->settings == NULL && mLastRequestSettings.isEmpty()) {
        ALOGE("First request does not have metadata");
        return BAD_VALUE;
    }

    if(request->input_buffer) {
        /* Ignore input buffer */
        /* TODO: do we expect any input buffer? */
        request->input_buffer->release_fence = -1;
    }

    if(!request->settings) {
        cm.acquire(mLastRequestSettings);
    } else {
        cm = request->settings;
    }

    /* Apply exposure/gain from request metadata — only with soft ISP */
    if (mSoftIspEnabled) {
        int32_t exposureUs = mSensorCfg.exposureDefault;
        if (cm.exists(ANDROID_SENSOR_EXPOSURE_TIME)) {
            int64_t exposureNs = *cm.find(ANDROID_SENSOR_EXPOSURE_TIME).data.i64;
            exposureUs = (int32_t)(exposureNs / 1000);
        }
        /* EV compensation applied on top of exposure time */
        if (cm.exists(ANDROID_CONTROL_AE_EXPOSURE_COMPENSATION)) {
            int32_t evComp = *cm.find(ANDROID_CONTROL_AE_EXPOSURE_COMPENSATION).data.i32;
            if (evComp > 0)
                for (int i = 0; i < evComp; i++) exposureUs = exposureUs * 5 / 4;
            else
                for (int i = 0; i < -evComp; i++) exposureUs = exposureUs * 4 / 5;
        }
        if (exposureUs < 100) exposureUs = 100;
        if (exposureUs > 200000) exposureUs = 200000;

        /* Split into exposure + gain to preserve FPS */
        int32_t actualExposure, extraGainQ8;
        mSensorCfg.splitExposureGain(exposureUs, &actualExposure, &extraGainQ8);
        mDev->setControl(V4L2_CID_EXPOSURE, actualExposure);

        int32_t gain = mSensorCfg.gainUnit; /* 1x */
        if (cm.exists(ANDROID_SENSOR_SENSITIVITY))
            gain = mSensorCfg.isoToGain(*cm.find(ANDROID_SENSOR_SENSITIVITY).data.i32);
        gain = (int32_t)((int64_t)gain * extraGainQ8 / 256);
        if (gain < 1) gain = 1;
        if (gain > mSensorCfg.gainMax) gain = mSensorCfg.gainMax;
        mDev->setControl(V4L2_CID_GAIN, gain);
    }

    /* Focus control — only with soft ISP */
    uint8_t afMode = ANDROID_CONTROL_AF_MODE_OFF;
    if (!mSoftIspEnabled) goto skip_focus;
    if (cm.exists(ANDROID_CONTROL_AF_MODE))
        afMode = *cm.find(ANDROID_CONTROL_AF_MODE).data.u8;

    /* Manual focus: LENS_FOCUS_DISTANCE (diopters = 1/meters) → VCM position */
    if (afMode == ANDROID_CONTROL_AF_MODE_OFF && cm.exists(ANDROID_LENS_FOCUS_DISTANCE)) {
        float diopter = *cm.find(ANDROID_LENS_FOCUS_DISTANCE).data.f;
        /* Map: 0 diopters (infinity) → 140, 10 diopters (10cm) → 640 */
        int32_t pos = 140 + (int32_t)(diopter * 50.0f);
        if (pos < 0) pos = 0;
        if (pos > 1023) pos = 1023;
        mDev->setFocusPosition(pos);
        mFocusPosition = pos;
        mAfSweepActive = false;
    }

    /* AF trigger — start two-pass contrast-detect sweep */
    if (cm.exists(ANDROID_CONTROL_AF_TRIGGER)) {
        uint8_t trigger = *cm.find(ANDROID_CONTROL_AF_TRIGGER).data.u8;
        if (trigger == ANDROID_CONTROL_AF_TRIGGER_START && !mAfSweepActive) {
            mAfSweepActive = true;
            mAfFinePass = false;
            mAfSettleFrames = 0;
            if (afMode == ANDROID_CONTROL_AF_MODE_MACRO) {
                mAfSweepPos = 400; mAfSweepEnd = 650; mAfSweepStep = 25;
            } else {
                mAfSweepPos = 140; mAfSweepEnd = 640; mAfSweepStep = 25;
            }
            mAfSweepBestPos = mAfSweepPos;
            mAfSweepBestScore = 0;
            ALOGD("AF coarse sweep started (mode=%d, %d→%d step %d)",
                  afMode, mAfSweepPos, mAfSweepEnd, mAfSweepStep);
        } else if (trigger == ANDROID_CONTROL_AF_TRIGGER_CANCEL) {
            mAfSweepActive = false;
        }
    }

    /* Continuous AF: re-trigger every ~60 frames if not sweeping */
    if ((afMode == ANDROID_CONTROL_AF_MODE_CONTINUOUS_PICTURE) &&
        !mAfSweepActive && (request->frame_number % 60 == 0)) {
        mAfSweepActive = true;
        mAfFinePass = false;
        mAfSettleFrames = 0;
        mAfSweepPos = 140; mAfSweepEnd = 640; mAfSweepStep = 25;
        mAfSweepBestPos = mFocusPosition;
        mAfSweepBestScore = 0;
    }

skip_focus:
    notifyShutter(request->frame_number, (uint64_t)timestamp);

    int64_t t0 = systemTime();
    BENCHMARK_SECTION("Lock/Read") {
        frame = mDev->readLock();
    }
    int64_t t1 = systemTime();

    if(!frame) {
        return NOT_ENOUGH_DATA;
    }

    /* AF sweep: step focus position each frame, measure contrast */
    if (mSoftIspEnabled && mAfSweepActive) {
        mDev->setFocusPosition(mAfSweepPos);
    }

    buffers.setCapacity(request->num_output_buffers);

    /* Parse crop region for digital zoom */
    int cropX = 0, cropY = 0, cropW = res.width, cropH = res.height;
    if (cm.exists(ANDROID_SCALER_CROP_REGION)) {
        auto crop = cm.find(ANDROID_SCALER_CROP_REGION).data.i32;
        /* crop is [x, y, width, height] in sensor coordinates */
        auto sensor = mDev->sensorResolution();
        /* Map from sensor coordinates to V4L2 resolution */
        cropX = crop[0] * (int)res.width / (int)sensor.width;
        cropY = crop[1] * (int)res.height / (int)sensor.height;
        cropW = crop[2] * (int)res.width / (int)sensor.width;
        cropH = crop[3] * (int)res.height / (int)sensor.height;
        /* Clamp */
        if (cropX < 0) cropX = 0;
        if (cropY < 0) cropY = 0;
        if (cropW < 16) cropW = 16;
        if (cropH < 16) cropH = 16;
        if (cropX + cropW > (int)res.width) cropW = res.width - cropX;
        if (cropY + cropH > (int)res.height) cropH = res.height - cropY;
    }
    bool needZoom = (cropW != (int)res.width || cropH != (int)res.height);

    uint8_t *rgbaBuffer = NULL;
    for(size_t i = 0; i < request->num_output_buffers; ++i) {
        const camera3_stream_buffer &srcBuf = request->output_buffers[i];
        uint8_t *buf = NULL;

        sp<Fence> acquireFence = new Fence(srcBuf.acquire_fence);
        e = acquireFence->wait(1000); /* FIXME: magic number */
        if(e == TIMED_OUT) {
            ALOGE("buffer %p  frame %-4u  Wait on acquire fence timed out", srcBuf.buffer, request->frame_number);
        }
        if(e == NO_ERROR) {
            const Rect rect((int)srcBuf.stream->width, (int)srcBuf.stream->height);
            e = GraphicBufferMapper::get().lock(*srcBuf.buffer, GRALLOC_USAGE_SW_WRITE_OFTEN, rect, (void **)&buf);
            if(e != NO_ERROR) {
                ALOGE("buffer %p  frame %-4u  lock failed", srcBuf.buffer, request->frame_number);
            }
        }
        if(e != NO_ERROR) {
            do GraphicBufferMapper::get().unlock(*request->output_buffers[i].buffer); while(i--);
            return NO_INIT;
        }

        switch(srcBuf.stream->format) {
            case HAL_PIXEL_FORMAT_RGBA_8888: {
                unsigned streamW = srcBuf.stream->width;
                unsigned streamH = srcBuf.stream->height;

                if(!rgbaBuffer) {
                    BENCHMARK_SECTION("Raw->RGBA") {
                        if(frame->pixFmt == V4L2_PIX_FMT_UYVY) {
                            mConverter.UYVYToRGBA(frame->buf, needZoom ? mRgbaTemp : buf, res.width, res.height);
                            rgbaBuffer = needZoom ? mRgbaTemp : buf;
                        } else if(frame->pixFmt == V4L2_PIX_FMT_YUYV) {
                            mConverter.YUY2ToRGBA(frame->buf, needZoom ? mRgbaTemp : buf, res.width, res.height);
                            rgbaBuffer = needZoom ? mRgbaTemp : buf;
                        } else if(!needZoom && res.width == streamW && res.height == streamH) {
                            /* Try GPU direct render to gralloc — no CPU readback */
                            GraphicBufferMapper::get().unlock(*srcBuf.buffer);
                            sp<GraphicBuffer> gb = new GraphicBuffer(streamW, streamH,
                                HAL_PIXEL_FORMAT_RGBA_8888,
                                GRALLOC_USAGE_HW_RENDER | GRALLOC_USAGE_HW_COMPOSER,
                                streamW, const_cast<native_handle_t *>(*srcBuf.buffer),
                                false);
                            if (mIsp->processToGralloc(frame->buf, gb->getNativeBuffer(),
                                                        res.width, res.height, streamW, streamH,
                                                        frame->pixFmt)) {
                                /* Success — GPU wrote directly.
                                 * Re-lock as READ to sync GPU writes for compositor, then unlock releases it */
                                const Rect rect((int)streamW, (int)streamH);
                                GraphicBufferMapper::get().lock(*srcBuf.buffer,
                                    GRALLOC_USAGE_SW_READ_OFTEN, rect, (void **)&buf);
                                rgbaBuffer = buf;
                            } else {
                                /* Failed — re-lock and fall back to CPU readback */
                                const Rect rect2((int)streamW, (int)streamH);
                                GraphicBufferMapper::get().lock(*srcBuf.buffer,
                                    GRALLOC_USAGE_SW_WRITE_OFTEN, rect2, (void **)&buf);
                                mIsp->processFromDmabuf(frame->dmabufFd, frame->buf, buf,
                                                         res.width, res.height, frame->pixFmt);
                                rgbaBuffer = buf;
                            }
                        } else {
                            /* Zoom or size mismatch — CPU readback path */
                            uint8_t *convDst = needZoom ? mRgbaTemp : buf;
                            if (!needZoom && (res.width != streamW || res.height != streamH))
                                convDst = mRgbaTemp;
                            mIsp->processFromDmabuf(frame->dmabufFd, frame->buf, convDst,
                                                     res.width, res.height, frame->pixFmt);
                            rgbaBuffer = convDst;
                        }
                    }
                }

                BENCHMARK_SECTION("Zoom/Copy") {
                    if (needZoom) {
                        /* Crop + scale: pointer offset for crop, ARGBScale for zoom */
                        const uint8_t *cropSrc = rgbaBuffer + (cropY * res.width + cropX) * 4;
                        libyuv::ARGBScale(cropSrc, res.width * 4, cropW, cropH,
                                          buf, streamW * 4, streamW, streamH,
                                          libyuv::kFilterBilinear);
                    } else if (rgbaBuffer != buf) {
                        /* Resolution mismatch: copy/crop */
                        unsigned copyW = (streamW < res.width) ? streamW : res.width;
                        unsigned copyH = (streamH < res.height) ? streamH : res.height;
                        for (unsigned y = 0; y < copyH; y++)
                            memcpy(buf + y * streamW * 4,
                                   rgbaBuffer + y * res.width * 4,
                                   copyW * 4);
                    }
                }
                break;
            }
            case HAL_PIXEL_FORMAT_BLOB: {
                BENCHMARK_SECTION("YUV->JPEG") {
                    const size_t maxImageSize = mJpegBufferSize - sizeof(camera3_jpeg_blob);
                    uint8_t jpegQuality = 95;
                    if(cm.exists(ANDROID_JPEG_QUALITY)) {
                        jpegQuality = *cm.find(ANDROID_JPEG_QUALITY).data.u8;
                    }
                    ALOGD("JPEG quality = %u", jpegQuality);

                    uint8_t *bufEnd = NULL;
                    if(frame->pixFmt == V4L2_PIX_FMT_UYVY)
                        bufEnd = mConverter.UYVYToJPEG(frame->buf, buf, res.width, res.height, maxImageSize, jpegQuality);
                    else if(frame->pixFmt == V4L2_PIX_FMT_YUYV)
                        bufEnd = mConverter.YUY2ToJPEG(frame->buf, buf, res.width, res.height, maxImageSize, jpegQuality);
                    else {
                        /* Bayer: demosaic to mRgbaTemp (persistent), then encode JPEG.
                         * Must use mRgbaTemp — Vulkan double buffering holds mPrevDst
                         * across frames, so one-shot allocations cause use-after-free. */
                        mIsp->process(frame->buf, mRgbaTemp, res.width, res.height, frame->pixFmt);
                        bufEnd = ImageConverter::RGBAToJPEG(mRgbaTemp, buf,
                            res.width, res.height, maxImageSize, jpegQuality);
                    }

                    if(bufEnd != buf) {
                        camera3_jpeg_blob *jpegBlob = reinterpret_cast<camera3_jpeg_blob*>(buf + maxImageSize);
                        jpegBlob->jpeg_blob_id  = CAMERA3_JPEG_BLOB_ID;
                        jpegBlob->jpeg_size     = (uint32_t)(bufEnd - buf);
                    } else {
                        ALOGE("%s: JPEG image too big!", __FUNCTION__);
                    }
                }
                break;
            }
            default:
                ALOGE("Unknown pixel format %d in buffer %p (stream %p), ignoring", srcBuf.stream->format, srcBuf.buffer, srcBuf.stream);
        }
    }

    /* AF sweep: two-pass contrast-detect with VCM settling */
    if (mSoftIspEnabled && mAfSweepActive && rgbaBuffer) {
        /* Skip frames after VCM move to let lens settle */
        if (mAfSettleFrames > 0) {
            mAfSettleFrames--;
            goto af_done;
        }

        /* Compute sharpness: sum of absolute horizontal gradient in center 1/4 */
        unsigned cx = res.width / 4, cy = res.height / 4;
        unsigned cw = res.width / 2, ch = res.height / 2;
        uint64_t score = 0;
        for (unsigned y = cy; y < cy + ch; y += 4) {
            const uint8_t *row = rgbaBuffer + y * res.width * 4;
            for (unsigned x = cx + 1; x < cx + cw; x += 4) {
                int dg = (int)row[x*4+1] - (int)row[(x-1)*4+1]; /* green gradient */
                score += (uint64_t)(dg < 0 ? -dg : dg);
            }
        }

        if (score > mAfSweepBestScore) {
            mAfSweepBestScore = score;
            mAfSweepBestPos = mAfSweepPos;
        }

        ALOGD("AF %s: pos=%d score=%llu best=%d/%llu",
              mAfFinePass ? "fine" : "coarse",
              mAfSweepPos, (unsigned long long)score,
              mAfSweepBestPos, (unsigned long long)mAfSweepBestScore);

        mAfSweepPos += mAfSweepStep;
        if (mAfSweepPos > mAfSweepEnd) {
            if (!mAfFinePass) {
                /* Coarse pass done — start fine pass around best position */
                mAfFinePass = true;
                mAfSweepStep = 5;
                mAfSweepPos = mAfSweepBestPos - 25;
                if (mAfSweepPos < 0) mAfSweepPos = 0;
                mAfSweepEnd = mAfSweepBestPos + 25;
                if (mAfSweepEnd > 1023) mAfSweepEnd = 1023;
                mAfSweepBestScore = 0;
                ALOGD("AF fine sweep: %d→%d step %d",
                      mAfSweepPos, mAfSweepEnd, mAfSweepStep);
            } else {
                /* Fine pass done — go to best position */
                mDev->setFocusPosition(mAfSweepBestPos);
                mFocusPosition = mAfSweepBestPos;
                mAfSweepActive = false;
                ALOGD("AF done: best=%d score=%llu", mAfSweepBestPos,
                      (unsigned long long)mAfSweepBestScore);
                goto af_done;
            }
        }

        /* Move VCM and wait 1 frame for settling */
        mDev->setFocusPosition(mAfSweepPos);
        mAfSettleFrames = 1;
    }
af_done:

    /* Unlocking all buffers in separate loop allows to copy data from already processed buffer to not yet processed one */
    for(size_t i = 0; i < request->num_output_buffers; ++i) {
        const camera3_stream_buffer &srcBuf = request->output_buffers[i];

        GraphicBufferMapper::get().unlock(*srcBuf.buffer);
        buffers.push_back(srcBuf);
        buffers.editTop().acquire_fence = -1;
        buffers.editTop().release_fence = -1;
        buffers.editTop().status = CAMERA3_BUFFER_STATUS_OK;
    }

    int64_t t2 = systemTime();
    BENCHMARK_SECTION("Unlock") {
        mDev->unlock(frame);
    }
    ALOGD("PERF: dqbuf=%lldms convert=%lldms total=%lldms",
          (long long)(t1-t0)/1000000, (long long)(t2-t1)/1000000,
          (long long)(t2-t0)/1000000);

    int64_t sensorTimestamp = timestamp;
    int64_t syncFrameNumber = request->frame_number;

    cm.update(ANDROID_SENSOR_TIMESTAMP, &sensorTimestamp, 1);
    cm.update(ANDROID_SYNC_FRAME_NUMBER, &syncFrameNumber, 1);

    /* Report AF state + focus distance */
    uint8_t afState = mAfSweepActive ?
        ANDROID_CONTROL_AF_STATE_ACTIVE_SCAN :
        ANDROID_CONTROL_AF_STATE_FOCUSED_LOCKED;
    cm.update(ANDROID_CONTROL_AF_STATE, &afState, 1);

    float reportDiopter = (mFocusPosition - 140) / 50.0f;
    if (reportDiopter < 0) reportDiopter = 0;
    cm.update(ANDROID_LENS_FOCUS_DISTANCE, &reportDiopter, 1);

    auto result = cm.getAndLock();
    processCaptureResult(request->frame_number, result, buffers);
    cm.unlock(result);

    // Cache the settings for next time
    mLastRequestSettings.acquire(cm);

    /* Print stats */
    char bmOut[1024];
    BENCHMARK_STRING(bmOut, sizeof(bmOut), 6);
    ALOGV("    time (avg):  %s", bmOut);

    return NO_ERROR;
}

inline void Camera::notifyShutter(uint32_t frameNumber, uint64_t timestamp) {
    camera3_notify_msg_t msg;
    msg.type = CAMERA3_MSG_SHUTTER;
    msg.message.shutter.frame_number = frameNumber;
    msg.message.shutter.timestamp = timestamp;
    mCallbackOps->notify(mCallbackOps, &msg);
}

void Camera::processCaptureResult(uint32_t frameNumber, const camera_metadata_t *result, const Vector<camera3_stream_buffer> &buffers) {
    camera3_capture_result captureResult;
    captureResult.frame_number = frameNumber;
    captureResult.result = result;
    captureResult.num_output_buffers = buffers.size();
    captureResult.output_buffers = buffers.array();
    captureResult.input_buffer = NULL;
    captureResult.partial_result = 0;

    mCallbackOps->process_capture_result(mCallbackOps, &captureResult);
}

/******************************************************************************\
                                STATIC WRAPPERS
\******************************************************************************/

int Camera::sClose(hw_device_t *device) {
    /* TODO: check device module */
    Camera *thiz = static_cast<Camera *>(reinterpret_cast<camera3_device_t *>(device));
    return thiz->closeDevice();
}

int Camera::sInitialize(const camera3_device *device, const camera3_callback_ops_t *callback_ops) {
    /* TODO: check pointers */
    Camera *thiz = static_cast<Camera *>(const_cast<camera3_device *>(device));
    return thiz->initialize(callback_ops);
}

int Camera::sConfigureStreams(const camera3_device *device, camera3_stream_configuration_t *stream_list) {
    /* TODO: check pointers */
    Camera *thiz = static_cast<Camera *>(const_cast<camera3_device *>(device));
    return thiz->configureStreams(stream_list);
}

int Camera::sRegisterStreamBuffers(const camera3_device *device, const camera3_stream_buffer_set_t *buffer_set) {
    /* TODO: check pointers */
    Camera *thiz = static_cast<Camera *>(const_cast<camera3_device *>(device));
    return thiz->registerStreamBuffers(buffer_set);
}

const camera_metadata_t * Camera::sConstructDefaultRequestSettings(const camera3_device *device, int type) {
    /* TODO: check pointers */
    Camera *thiz = static_cast<Camera *>(const_cast<camera3_device *>(device));
    return thiz->constructDefaultRequestSettings(type);
}

int Camera::sProcessCaptureRequest(const camera3_device *device, camera3_capture_request_t *request) {
    /* TODO: check pointers */
    Camera *thiz = static_cast<Camera *>(const_cast<camera3_device *>(device));
    return thiz->processCaptureRequest(request);
}

void Camera::sGetMetadataVendorTagOps(const camera3_device *device, vendor_tag_query_ops_t *ops) {
    /* TODO: implement */
    ALOGD("%s: IMPLEMENT ME!", __FUNCTION__);
}

void Camera::sDump(const camera3_device *device, int fd) {
    /* TODO: implement */
    ALOGD("%s: IMPLEMENT ME!", __FUNCTION__);
}

int Camera::sFlush(const camera3_device *device) {
    return OK;
}

camera3_device_ops_t Camera::sOps = {
    .initialize                         = Camera::sInitialize,
    .configure_streams                  = Camera::sConfigureStreams,
    .register_stream_buffers            = Camera::sRegisterStreamBuffers,
    .construct_default_request_settings = Camera::sConstructDefaultRequestSettings,
    .process_capture_request            = Camera::sProcessCaptureRequest,
    .get_metadata_vendor_tag_ops        = Camera::sGetMetadataVendorTagOps,
    .dump                               = Camera::sDump,
    .flush                              = Camera::sFlush,
    .reserved = {0}
};

}; /* namespace android */
