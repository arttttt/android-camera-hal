/*
 * Copyright (C) 2026 Artem Bambalov
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

#define LOG_TAG "Cam-Camera3Device"
#define LOG_NDEBUG NDEBUG

#include "Camera3Device.h"

#include <assert.h>
#include <stdio.h>

#include <cutils/properties.h>
#include <utils/Log.h>
#include <utils/misc.h>

#include "Camera3Request.h"
#include "Camera3ResultSink.h"
#include "DbgUtils.h"
#include "metadata/CameraStaticMetadata.h"
#include "metadata/RequestTemplateBuilder.h"
#include "StreamConfig.h"

extern camera_module_t HAL_MODULE_INFO_SYM;

namespace android {

namespace {

/* Mi Pad 1: the back camera is the IMX179 module by Primax, the front
 * one the OV5693 by Sunny. Board knowledge, here where the cameras are
 * told apart by facing. */
mocha::camera::Session::Config sessionConfig(const char *devNode, int facing) {
    mocha::camera::Session::Config c;
    c.devNode     = devNode;
    c.facingFront = facing == CAMERA_FACING_FRONT;
    c.sensorName  = c.facingFront ? "OV5693" : "IMX179";
    c.integrator  = c.facingFront ? "Sunny"  : "Primax";

    /* Full ISP or demosaic only; read once per camera, so toggling
     * the property mid-session has no effect. */
    char propVal[PROPERTY_VALUE_MAX] = {0};
    property_get("persist.camera.soft_isp", propVal, "1");
    c.softIsp = propVal[0] == '1';

    /* Requests in flight; read once per camera, and cameraserver keeps
     * the characteristics it read: a change needs the camera provider
     * restarted. */
    c.requestsInFlight = property_get_int32("persist.camera.requests_in_flight",
                                            c.requestsInFlight);
    ALOGI("requests in flight: %d", c.requestsInFlight);
    c.writeDelayUs = property_get_int32("persist.camera.write_delay_us",
                                        c.writeDelayUs);
    if (c.writeDelayUs)
        ALOGI("sensor writes %d us after a frame's start", c.writeDelayUs);

    property_get("ro.product.manufacturer", propVal, "");
    c.make = propVal;
    property_get("ro.product.model", propVal, "");
    c.model = propVal;
    return c;
}

} /* namespace */

Camera3Device::Camera3Device(const char *devNode, int facing)
    : mFacing(facing) {
    DBGUTILS_AUTOLOGCALL(__func__);

    common.tag     = HARDWARE_DEVICE_TAG;
    common.version = CAMERA_DEVICE_API_VERSION_3_4;
    common.module  = &HAL_MODULE_INFO_SYM.common;
    common.close   = Camera3Device::sClose;
    ops            = &sOps;
    priv           = NULL;

    mResultSink.reset(new Camera3ResultSink(&mCallbackOps));
    mSession.reset(new mocha::camera::Session(sessionConfig(devNode, facing),
                                              mResultSink.get(), &mGralloc));
}

Camera3Device::~Camera3Device() {
    DBGUTILS_AUTOLOGCALL(__func__);
    mSession.reset();
}

status_t Camera3Device::cameraInfo(struct camera_info *info) {
    DBGUTILS_AUTOLOGCALL(__func__);
    Mutex::Autolock lock(mMutex);
    info->facing = mFacing;
    info->orientation = mSession->sensorOrientationDegrees();
    info->device_version = CAMERA_DEVICE_API_VERSION_3_4;
    info->static_camera_characteristics = staticCharacteristics();
    return NO_ERROR;
}

int Camera3Device::openDevice(hw_device_t **device) {
    DBGUTILS_AUTOLOGCALL(__func__);
    Mutex::Autolock lock(mMutex);
    *device = &common;
    mResultSink->setFocalLength(mSession->capability().lens.focalLengthMm);
    if (!mSession->open())
        ALOGE("openDevice: the core did not open");
    return NO_ERROR;
}

int Camera3Device::closeDevice() {
    DBGUTILS_AUTOLOGCALL(__func__);

    /* The session stops before mMutex is taken: an engine may still be
     * handing a result back through a framework callback. Every frame
     * held goes back to the framework before close returns. */
    mSession->close();

    Mutex::Autolock lock(mMutex);
    mLastRequestSettings.clear();
    mStreams.clear();
    return NO_ERROR;
}

camera_metadata_t *Camera3Device::staticCharacteristics() {
    if (!mStaticCharacteristics)
        mStaticCharacteristics = CameraStaticMetadata::build(mSession->capability());
    return mStaticCharacteristics;
}

int Camera3Device::initialize(const camera3_callback_ops_t *callbackOps) {
    DBGUTILS_AUTOLOGCALL(__func__);
    Mutex::Autolock lock(mMutex);
    mCallbackOps = callbackOps;
    return NO_ERROR;
}

const camera_metadata_t *Camera3Device::constructDefaultRequestSettings(int type) {
    DBGUTILS_AUTOLOGCALL(__func__);
    Mutex::Autolock lock(mMutex);

    /* camera3.h: an unknown template type (vendor ones included) gets
     * NULL. mDefaultRequestSettings is indexed by type. */
    if (type < CAMERA3_TEMPLATE_PREVIEW || type >= CAMERA3_TEMPLATE_COUNT) {
        ALOGE("constructDefaultRequestSettings: unknown template %d", type);
        return NULL;
    }
    if (!mDefaultRequestSettings[type])
        mDefaultRequestSettings[type] = RequestTemplateBuilder::build(type, mSession->capability());
    return mDefaultRequestSettings[type];
}

int Camera3Device::configureStreams(camera3_stream_configuration_t *streamList) {
    DBGUTILS_AUTOLOGCALL(__func__);
    Mutex::Autolock lock(mMutex);

    for (size_t i = 0; i < streamList->num_streams; ++i) {
        const camera3_stream_t *s = streamList->streams[i];
        ALOGV("stream from framework: p=%p fmt=0x%.2x type=%u usage=0x%.8x %ux%u buf_no=%u",
              s, s->format, s->stream_type, s->usage, s->width, s->height, s->max_buffers);
    }

    {
        status_t e = StreamConfig::normalize(
                streamList, (uint32_t)mSession->capability().maxBuffersPerStream);
        if (e != NO_ERROR)
            return e;
    }

    /* The streams by the core's ids: their index in this configuration.
     * The framework sends no request across a configuration, so no
     * frame holds an id of the old table. */
    std::vector<std::pair<camera3_stream_t *, mocha::camera::StreamSpec>> streams;
    std::vector<mocha::camera::StreamSpec> specs;
    std::vector<camera3_stream_t *> ids;
    for (size_t i = 0; i < streamList->num_streams; ++i) {
        camera3_stream_t *s = streamList->streams[i];
        mocha::camera::StreamSpec spec;
        spec.id     = (mocha::camera::StreamId)i;
        spec.width  = s->width;
        spec.height = s->height;
        if (!toPixelFormat(s->format, &spec.format)) {
            ALOGE("configureStreams: stream %zu format 0x%x not produced", i, s->format);
            return BAD_VALUE;
        }
        streams.emplace_back(s, spec);
        specs.push_back(spec);
        ids.push_back(s);
    }

    /* The core picks the capture; streams no sensor mode feeds are
     * refused before anything changes, as camera3.h asks. */
    switch (mSession->configure(specs)) {
    case mocha::camera::Session::ConfigureResult::Ok:
        break;
    case mocha::camera::Session::ConfigureResult::Unsupported:
        return BAD_VALUE;
    case mocha::camera::Session::ConfigureResult::Failed:
        return NO_INIT;
    }
    mStreams = std::move(streams);
    mResultSink->setStreams(std::move(ids));
    return NO_ERROR;
}

int Camera3Device::processCaptureRequest(camera3_capture_request_t *request) {
    assert(request != NULL);

    FPSCOUNTER_HERE(120);
    ALOGV("--- capture request --- f=%-5u in_buf=%p  out_bufs=%p[%u] --- fps %4.1f (avg %4.1f)",
          request->frame_number, request->input_buffer,
          request->output_buffers, request->num_output_buffers,
          FPSCOUNTER_VALUE(1), FPSCOUNTER_VALUE());

    /* Acknowledge unused input buffer (reprocess slot is reserved but
     * not consumed yet). */
    if (request->input_buffer)
        request->input_buffer->release_fence = -1;

    mocha::camera::FrameRequest frame;
    frame.frameNumber = request->frame_number;
    {
        Mutex::Autolock lock(mMutex);

        if (!mSession->running()) {
            ALOGE("processCaptureRequest: the core is not running");
            notifyError(request->frame_number, NULL, CAMERA3_MSG_ERROR_REQUEST);
            return INVALID_OPERATION;
        }

        if (request->settings == NULL && mLastRequestSettings.isEmpty()) {
            ALOGE("First request does not have metadata");
            notifyError(request->frame_number, NULL, CAMERA3_MSG_ERROR_REQUEST);
            return BAD_VALUE;
        }

        /* The outputs, by the core's stream ids. A buffer of a stream
         * that was not configured makes the request invalid; nothing of
         * it has been taken yet. */
        frame.outputs.resize(request->num_output_buffers);
        bool blob = false;
        for (uint32_t i = 0; i < request->num_output_buffers; ++i) {
            const camera3_stream_buffer &src = request->output_buffers[i];
            const mocha::camera::StreamSpec *spec = nullptr;
            for (const auto &entry : mStreams)
                if (entry.first == src.stream) spec = &entry.second;
            if (!spec) {
                ALOGE("frame %u: buffer of an unconfigured stream %p",
                      request->frame_number, src.stream);
                return -EINVAL;
            }
            frame.outputs[i].stream = *spec;
            frame.outputs[i].buffer =
                reinterpret_cast<mocha::camera::NativeBuffer *>(src.buffer);
            if (spec->format == mocha::camera::PixelFormat::Blob) blob = true;
        }
        /* Acquire-fence ownership transfers to the core from here. */
        for (uint32_t i = 0; i < request->num_output_buffers; ++i)
            frame.outputs[i].acquireFence = request->output_buffers[i].acquire_fence;

        /* A request with settings=NULL repeats the last ones. */
        if (request->settings)
            mLastRequestSettings = request->settings;
        frame.settings = parseFrameSettings(mLastRequestSettings);
        if (blob) frame.settings.stillCapture = true;

        /* The echo the frame's results start from, before the core can
         * send any of them. */
        mResultSink->expect(frame.frameNumber, mLastRequestSettings);
    }

    mSession->submit(std::move(frame));
    return NO_ERROR;
}

int Camera3Device::flush() {
    DBGUTILS_AUTOLOGCALL(__func__);
    /* Without mMutex: process_capture_request may be called while
     * flush is in progress and must return quickly. */
    mSession->flush();
    return OK;
}

void Camera3Device::dump(int fd) {
    Mutex::Autolock lock(mMutex);
    dprintf(fd, "Camera3Device: facing %s, %zu streams configured\n",
            mFacing == CAMERA_FACING_FRONT ? "front" : "back", mStreams.size());
    for (const auto &s : mStreams) {
        dprintf(fd, "  stream %u: %ux%u format %d\n", s.second.id,
                s.second.width, s.second.height, (int)s.second.format);
    }
    mSession->dump(fd);
}

void Camera3Device::notifyError(uint32_t frameNumber, camera3_stream_t *stream, int errorCode) {
    if (!mCallbackOps) return;
    camera3_notify_msg_t msg;
    msg.type = CAMERA3_MSG_ERROR;
    msg.message.error.frame_number = frameNumber;
    msg.message.error.error_stream = stream;
    msg.message.error.error_code   = errorCode;
    mCallbackOps->notify(mCallbackOps, &msg);
}

/******************************************************************************\
                                STATIC WRAPPERS
\******************************************************************************/

namespace {

Camera3Device *deviceOf(const camera3_device *device) {
    return static_cast<Camera3Device *>(const_cast<camera3_device *>(device));
}

} /* namespace */

int Camera3Device::sClose(hw_device_t *device) {
    if (!device) {
        ALOGE("sClose: NULL device");
        return -ENODEV;
    }
    return deviceOf(reinterpret_cast<camera3_device_t *>(device))->closeDevice();
}

int Camera3Device::sInitialize(const camera3_device *device, const camera3_callback_ops_t *callback_ops) {
    if (!device) return -ENODEV;
    return deviceOf(device)->initialize(callback_ops);
}

int Camera3Device::sConfigureStreams(const camera3_device *device, camera3_stream_configuration_t *stream_list) {
    if (!device) return -ENODEV;
    return deviceOf(device)->configureStreams(stream_list);
}

const camera_metadata_t *Camera3Device::sConstructDefaultRequestSettings(const camera3_device *device, int type) {
    if (!device) return nullptr;
    return deviceOf(device)->constructDefaultRequestSettings(type);
}

int Camera3Device::sProcessCaptureRequest(const camera3_device *device, camera3_capture_request_t *request) {
    if (!device) return -ENODEV;
    return deviceOf(device)->processCaptureRequest(request);
}

void Camera3Device::sDump(const camera3_device *device, int fd) {
    if (!device) return;
    deviceOf(device)->dump(fd);
}

int Camera3Device::sFlush(const camera3_device *device) {
    /* Camera3Device is the camera3_device, like every other entry point
     * here; priv is never set. */
    if (!device) return -ENODEV;
    return deviceOf(device)->flush();
}

camera3_device_ops_t Camera3Device::sOps = {
    .initialize                         = Camera3Device::sInitialize,
    .configure_streams                  = Camera3Device::sConfigureStreams,
    /* Deprecated since HAL3.2 — the framework now hands gralloc
     * buffers to process_capture_request directly without a
     * pre-registration phase. Setting these non-NULL on a 3.2+
     * device causes Camera3Stream::finishConfiguration to bail
     * with -ENOSYS, which surfaces as
     * "configureStreamsLocked: Can't finish configuring output
     * stream" and leaves the session unable to start preview. */
    .register_stream_buffers            = NULL,
    .construct_default_request_settings = Camera3Device::sConstructDefaultRequestSettings,
    .process_capture_request            = Camera3Device::sProcessCaptureRequest,
    .get_metadata_vendor_tag_ops        = NULL,
    .dump                               = Camera3Device::sDump,
    .flush                              = Camera3Device::sFlush,
    .reserved = {0}
};

} /* namespace android */
