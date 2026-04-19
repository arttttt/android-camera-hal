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
#include <libyuv/rotate_argb.h>
#include <cutils/properties.h>
#include <vector>

/* Tegra camera CIDs — not in standard V4L2 headers */
#ifndef V4L2_CID_FRAME_LENGTH
#define V4L2_CID_FRAME_LENGTH   (V4L2_CTRL_CLASS_CAMERA | 0x2000)
#endif

#include "DbgUtils.h"
#include "Camera.h"
#include "metadata/CameraStaticMetadata.h"
#include "metadata/RequestTemplateBuilder.h"
#include "metadata/ResultMetadataBuilder.h"
#include "ImageConverter.h"
#include "IspPipeline.h"
#include "sensor/IspCalibration.h"

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
    , mSoftIspEnabled(true) {
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
    mAf = NULL;
    mExposure = NULL;
    mJpeg = NULL;
    mBufferProcessor = NULL;
    mDev = new V4l2Device(devNode);
    if(!mDev) {
        mValid = false;
    }
}

Camera::~Camera() {
    DBGUTILS_AUTOLOGCALL(__func__);
    gWorkers.stop();
    delete mBufferProcessor;
    delete mAf;
    delete mExposure;
    delete mJpeg;
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
    mStaticCharacteristics = CameraStaticMetadata::build(mDev, mFacing, &mJpegBufferSize);
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

    if(mDefaultRequestSettings[type])
        return mDefaultRequestSettings[type];
    mDefaultRequestSettings[type] = RequestTemplateBuilder::build(type, mDev, mFacing);
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

        /* Save original usage before overwrite — framework sets HW_VIDEO_ENCODER for video streams */
        uint32_t origUsage = newStream->usage;
        ALOGD("Stream[%zu]: %ux%u fmt=0x%x usage=0x%x", i,
              newStream->width, newStream->height, newStream->format, origUsage);

        switch(newStream->stream_type) {
            case CAMERA3_STREAM_OUTPUT:         newStream->usage = GRALLOC_USAGE_SW_WRITE_OFTEN;                                break;
            case CAMERA3_STREAM_INPUT:          newStream->usage = GRALLOC_USAGE_SW_READ_OFTEN;                                 break;
            case CAMERA3_STREAM_BIDIRECTIONAL:  newStream->usage = GRALLOC_USAGE_SW_WRITE_OFTEN | GRALLOC_USAGE_SW_READ_OFTEN;  break;
        }
        newStream->max_buffers = 4;

        /*
         * V4L2 resolution selection:
         * - If a video stream exists (HW_VIDEO_ENCODER), use its resolution
         *   so the sensor switches to the matching FPS mode (e.g. 720p@90fps).
         * - Otherwise use the largest non-BLOB stream.
         */
        if(newStream->format != HAL_PIXEL_FORMAT_BLOB) {
            bool isVideo = (origUsage & GRALLOC_USAGE_HW_VIDEO_ENCODER) != 0;
            if (isVideo) {
                /* Video stream takes priority */
                width = newStream->width;
                height = newStream->height;
                ALOGD("Video stream detected: %ux%u", width, height);
            } else if (!width || !height ||
                       (newStream->width * newStream->height > width * height)) {
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

    char propVal[PROPERTY_VALUE_MAX] = {0};
    property_get("persist.camera.soft_isp", propVal, "1");
    mSoftIspEnabled = (propVal[0] == '1');

    if (mAf) { delete mAf; mAf = NULL; }
    if (mIsp) { mIsp->destroy(); delete mIsp; mIsp = NULL; }
    mIsp = createIspPipeline();
    if (!mIsp->init()) {
        ALOGE("ISP init failed");
        delete mIsp; mIsp = NULL;
        return NO_INIT;
    }
    mIsp->setEnabled(mSoftIspEnabled);
    mIsp->setCcm((mFacing == CAMERA_FACING_BACK) ? IspCalibration::ccmImx179()
                                                  : IspCalibration::ccmOv5693());

    /* AF sweep + VCM control are only meaningful with the soft ISP
     * (which owns the AWB lock and whose RGBA output the sharpness
     * metric runs on). On the HW ISP path mAf stays null. */
    if (mSoftIspEnabled)
        mAf = new AutoFocusController(mDev, mIsp);

    if (mJpeg) { delete mJpeg; mJpeg = NULL; }
    mJpeg = new JpegEncoder(mIsp, &mConverter);

    if (mBufferProcessor) { delete mBufferProcessor; mBufferProcessor = NULL; }
    BufferProcessor::Deps bpDeps;
    bpDeps.isp       = mIsp;
    bpDeps.converter = &mConverter;
    bpDeps.jpeg      = mJpeg;
    bpDeps.af        = mAf;
    mBufferProcessor = new BufferProcessor(bpDeps);

    ALOGD("V4L2 target resolution: %ux%u, soft_isp=%d",
          width, height, mSoftIspEnabled);

    if(!mDev->setStreaming(false)) {
        ALOGE("Could not stop streaming");
        return NO_INIT;
    }

    /* Allocate Vulkan input buffers at the target resolution before V4L2 so
     * we can hand their dma-buf fds to VIDIOC_REQBUFS(DMABUF). If any step of
     * the export fails we fall back to MMAP + memcpy silently. */
    mIsp->prewarm(width, height, mDev->pixelFormat());
    {
        int n = mIsp->inputBufferCount();
        if (n > 0 && n == V4L2DEVICE_BUF_COUNT) {
            int fds[V4L2DEVICE_BUF_COUNT];
            bool allOk = true;
            for (int i = 0; i < n; i++) {
                fds[i] = mIsp->exportInputBufferFd(i);
                if (fds[i] < 0) { allOk = false; break; }
            }
            if (allOk) {
                mDev->setDmaBufFds(fds, n);
                ALOGD("V4L2 input: DMABUF mode, %d slots", n);
            } else {
                for (int i = 0; i < n; i++) if (fds[i] >= 0) ::close(fds[i]);
                ALOGW("V4L2 input: DMABUF export failed, falling back to MMAP");
            }
        }
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

    /* Exposure/gain are only driven by the HAL on the soft ISP path —
     * the HW ISP firmware owns those controls otherwise. */
    if (mExposure) { delete mExposure; mExposure = NULL; }
    if (mSoftIspEnabled) {
        mExposure = new ExposureControl(mDev, mSensorCfg);
        mExposure->applyDefaults();
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
        notifyError(request->frame_number, NULL, CAMERA3_MSG_ERROR_REQUEST);
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

    /* Actual applied exposure/gain — echoed back in result metadata.
     * Seeded from sensor defaults so the HW-ISP path (which leaves
     * mExposure null) still reports sane values. */
    int32_t appliedExposureUs = mSensorCfg.exposureDefault;
    int32_t appliedGain       = mSensorCfg.gainDefault;
    if (mExposure) {
        mExposure->onSettings(cm);
        ExposureControl::Report r = mExposure->report();
        appliedExposureUs = r.appliedExposureUs;
        appliedGain       = r.appliedGain;
    }

    if (mAf)
        mAf->onSettings(cm, request->frame_number);

    notifyShutter(request->frame_number, (uint64_t)timestamp);

    int64_t t0 = systemTime();

    /* Drain the previous frame's GPU work before V4L2 can reuse any input
     * buffer slot (readLock flushes deferred QBUFs internally). Without
     * this, VI starts overwriting a slot the shader is still reading. */
    BENCHMARK_SECTION("GPU drain") {
        mIsp->waitForPreviousFrame();
    }

    BENCHMARK_SECTION("Lock/Read") {
        frame = mDev->readLock();
    }
    int64_t t1 = systemTime();

    if(!frame) {
        notifyError(request->frame_number, NULL, CAMERA3_MSG_ERROR_REQUEST);
        return NOT_ENOUGH_DATA;
    }

    if (mAf)
        mAf->onFrameStart();

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
    /* Per-output-buffer state for the zero-copy path:
     *   needsFinalUnlock — whether the cleanup loop at the bottom should
     *                      call GraphicBufferMapper::unlock on this buffer.
     *                      False iff we successfully handed the buffer back
     *                      unlocked via processToGralloc (framework will
     *                      wait on the release fence).
     *   releaseFd        — sync_fence fd returned by vkQueueSignalReleaseImageANDROID;
     *                      propagated as camera3_stream_buffer.release_fence. */
    std::vector<bool> needsFinalUnlock(request->num_output_buffers, true);
    std::vector<int>  releaseFd(request->num_output_buffers, -1);

    BufferProcessor::FrameContext fctx;
    fctx.frameBuf       = frame->buf;
    fctx.frameSlotIdx   = (frame->buf == NULL) ? frame->index : -1;
    fctx.pixFmt         = frame->pixFmt;
    fctx.resW           = res.width;
    fctx.resH           = res.height;
    fctx.cropX          = cropX;
    fctx.cropY          = cropY;
    fctx.cropW          = cropW;
    fctx.cropH          = cropH;
    fctx.needZoom       = needZoom;
    fctx.jpegBufferSize = mJpegBufferSize;
    fctx.rgbaScratch    = mRgbaTemp;

    for(size_t i = 0; i < request->num_output_buffers; ++i) {
        BufferProcessor::OutputState bpState;
        e = mBufferProcessor->processOne(request->output_buffers[i], fctx, cm,
                                          request->frame_number,
                                          &bpState, &rgbaBuffer);
        if (e != NO_ERROR) {
            do GraphicBufferMapper::get().unlock(*request->output_buffers[i].buffer); while(i--);
            notifyError(request->frame_number, NULL, CAMERA3_MSG_ERROR_REQUEST);
            return e;
        }
        needsFinalUnlock[i] = bpState.needsFinalUnlock;
        releaseFd[i]        = bpState.releaseFd;
    }

    if (mAf)
        mAf->onFrameData(rgbaBuffer, res.width, res.height);

    /* Unlocking all buffers in separate loop allows to copy data from already processed buffer to not yet processed one */
    for(size_t i = 0; i < request->num_output_buffers; ++i) {
        const camera3_stream_buffer &srcBuf = request->output_buffers[i];

        if (needsFinalUnlock[i])
            GraphicBufferMapper::get().unlock(*srcBuf.buffer);
        buffers.push_back(srcBuf);
        buffers.editTop().acquire_fence = -1;
        buffers.editTop().release_fence = releaseFd[i];
        buffers.editTop().status = CAMERA3_BUFFER_STATUS_OK;
    }

    int64_t t2 = systemTime();
    BENCHMARK_SECTION("Unlock") {
        mDev->unlock(frame);
    }
    ALOGD("PERF: dqbuf=%lldms convert=%lldms total=%lldms",
          (long long)(t1-t0)/1000000, (long long)(t2-t1)/1000000,
          (long long)(t2-t0)/1000000);

    ResultMetadataBuilder::FrameState fs;
    fs.timestampNs       = timestamp;
    fs.frameNumber       = request->frame_number;
    fs.appliedExposureUs = appliedExposureUs;
    fs.appliedGain       = appliedGain;
    fs.af.afMode       = ANDROID_CONTROL_AF_MODE_OFF;
    fs.af.afState      = ANDROID_CONTROL_AF_STATE_INACTIVE;
    fs.af.focusDiopter = 0.0f;
    if (mAf)
        fs.af = mAf->report();
    ResultMetadataBuilder::build(cm, fs, mSensorCfg);

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

void Camera::notifyError(uint32_t frameNumber, camera3_stream_t *stream, int errorCode) {
    camera3_notify_msg_t msg;
    msg.type = CAMERA3_MSG_ERROR;
    msg.message.error.frame_number = frameNumber;
    msg.message.error.error_stream = stream;
    msg.message.error.error_code   = errorCode;
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
