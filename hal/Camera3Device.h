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

#ifndef HAL_CAMERA3_DEVICE_H
#define HAL_CAMERA3_DEVICE_H

#include <memory>
#include <utility>
#include <vector>

#include <hardware/camera3.h>
#include <hardware/camera_common.h>
#include <utils/Errors.h>
#include <utils/Mutex.h>

#include "CameraMetadataCompat.h"
#include "Camera3GrallocPort.h"
#include "mochacamera/Buffers.h"
#include "mochacamera/Session.h"

namespace android {

class Camera3ResultSink;

/* One camera as Camera3 sees it: the camera3_device the framework
 * opens, over the core's Session.
 *
 * Everything Camera3 is here and nowhere else: the static
 * characteristics and request templates (from the core's Capability),
 * the stream configuration and its table of streams by the core's ids,
 * the parsing of each request's metadata into FrameSettings, and the
 * result sink and gralloc port the core answers through. */
class Camera3Device : public camera3_device {
public:
    Camera3Device(const char *devNode, int facing);
    ~Camera3Device();

    Camera3Device(const Camera3Device &) = delete;
    Camera3Device &operator=(const Camera3Device &) = delete;

    status_t cameraInfo(struct camera_info *info);

    int openDevice(hw_device_t **device);
    int closeDevice();

private:
    camera_metadata_t *staticCharacteristics();
    int initialize(const camera3_callback_ops_t *callbackOps);
    int configureStreams(camera3_stream_configuration_t *streamList);
    const camera_metadata_t *constructDefaultRequestSettings(int type);
    int processCaptureRequest(camera3_capture_request_t *request);
    int flush();
    void dump(int fd);

    void notifyError(uint32_t frameNumber, camera3_stream_t *stream, int errorCode);

    const int mFacing;
    const camera3_callback_ops_t *mCallbackOps = nullptr;

    camera_metadata_t *mStaticCharacteristics = nullptr;
    camera_metadata_t *mDefaultRequestSettings[CAMERA3_TEMPLATE_COUNT] = {};

    /* A request without settings repeats the last ones. */
    CameraMetadata mLastRequestSettings;

    /* The configured streams, by StreamId (their index here). */
    std::vector<std::pair<camera3_stream_t *, mocha::camera::StreamSpec>> mStreams;

    Camera3GrallocPort                         mGralloc;
    std::unique_ptr<Camera3ResultSink>         mResultSink;
    std::unique_ptr<mocha::camera::Session>    mSession;

    Mutex mMutex;

    static int sClose(hw_device_t *device);
    static int sInitialize(const struct camera3_device *device, const camera3_callback_ops_t *callback_ops);
    static int sConfigureStreams(const struct camera3_device *device, camera3_stream_configuration_t *stream_list);
    static const camera_metadata_t *sConstructDefaultRequestSettings(const struct camera3_device *device, int type);
    static int sProcessCaptureRequest(const struct camera3_device *device, camera3_capture_request_t *request);
    static void sDump(const struct camera3_device *device, int fd);
    static int sFlush(const struct camera3_device *device);

    static camera3_device_ops_t sOps;
};

} /* namespace android */

#endif /* HAL_CAMERA3_DEVICE_H */
