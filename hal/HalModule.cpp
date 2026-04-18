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

#define LOG_TAG "Cam-HalModule"

#include <hardware/camera_common.h>
#include <cutils/log.h>
#include <cstdlib>
#include <cstdio>
#include <cerrno>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <utils/Vector.h>

#include "Camera.h"

#define MAX_CAMERAS 8
#define MAX_VIDEO_NODES 16

extern camera_module_t HAL_MODULE_INFO_SYM;

namespace android {
namespace HalModule {

static Camera *sCameras[MAX_CAMERAS];
static int sNumCameras = 0;

static bool isV4l2CaptureDevice(const char *path) {
    int fd = open(path, O_RDWR | O_NONBLOCK);
    if (fd < 0)
        return false;

    struct v4l2_capability cap;
    bool result = false;
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0) {
        if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) {
            /* Check that it supports at least one format */
            struct v4l2_fmtdesc fmtDesc;
            memset(&fmtDesc, 0, sizeof(fmtDesc));
            fmtDesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            if (ioctl(fd, VIDIOC_ENUM_FMT, &fmtDesc) == 0)
                result = true;
        }
    }

    close(fd);
    return result;
}

static void enumerateCameras() {
    if (sNumCameras > 0)
        return;

    int backCount = 0;
    int frontCount = 0;

    for (int i = 0; i < MAX_VIDEO_NODES && sNumCameras < MAX_CAMERAS; i++) {
        char path[32];
        snprintf(path, sizeof(path), "/dev/video%d", i);

        if (!isV4l2CaptureDevice(path))
            continue;

        /* Default: first camera = back, rest = front */
        int facing = (sNumCameras == 0) ? CAMERA_FACING_BACK : CAMERA_FACING_FRONT;

        ALOGI("Found V4L2 capture device: %s (facing=%s)",
              path, facing == CAMERA_FACING_BACK ? "back" : "front");

        sCameras[sNumCameras] = new Camera(path, facing);
        if (sCameras[sNumCameras]->isValid()) {
            sNumCameras++;
        } else {
            ALOGW("Camera %s failed to initialize", path);
            delete sCameras[sNumCameras];
            sCameras[sNumCameras] = NULL;
        }
    }

    ALOGI("Enumerated %d camera(s)", sNumCameras);
}

static int getNumberOfCameras() {
    enumerateCameras();
    return sNumCameras;
}

static int getCameraInfo(int cameraId, struct camera_info *info) {
    enumerateCameras();
    if (cameraId < 0 || cameraId >= sNumCameras) {
        ALOGE("%s: invalid camera ID (%d)", __FUNCTION__, cameraId);
        return -ENODEV;
    }
    if (!sCameras[cameraId]->isValid()) {
        ALOGE("%s: camera %d is not initialized", __FUNCTION__, cameraId);
        return -ENODEV;
    }
    return sCameras[cameraId]->cameraInfo(info);
}

static int setCallbacks(const camera_module_callbacks_t * /*callbacks*/) {
    return OK;
}

static int openDevice(const hw_module_t *module, const char *name, hw_device_t **device) {
    if (module != &HAL_MODULE_INFO_SYM.common) {
        ALOGE("%s: invalid module (%p != %p)", __FUNCTION__, module, &HAL_MODULE_INFO_SYM.common);
        return -EINVAL;
    }
    if (name == NULL) {
        ALOGE("%s: NULL name", __FUNCTION__);
        return -EINVAL;
    }

    enumerateCameras();

    errno = 0;
    int cameraId = (int)strtol(name, NULL, 10);
    if (errno || cameraId < 0 || cameraId >= sNumCameras) {
        ALOGE("%s: invalid camera ID (%s)", __FUNCTION__, name);
        return -EINVAL;
    }
    if (!sCameras[cameraId]->isValid()) {
        ALOGE("%s: camera %d is not initialized", __FUNCTION__, cameraId);
        *device = NULL;
        return -ENODEV;
    }

    return sCameras[cameraId]->openDevice(device);
}

static struct hw_module_methods_t moduleMethods = {
    .open = openDevice
};

}; /* namespace HalModule */
}; /* namespace android */

camera_module_t HAL_MODULE_INFO_SYM = {
    .common = {
        .tag                = HARDWARE_MODULE_TAG,
        .module_api_version = CAMERA_MODULE_API_VERSION_2_3,
        .hal_api_version    = HARDWARE_HAL_API_VERSION,
        .id                 = CAMERA_HARDWARE_MODULE_ID,
        .name               = "V4l2 Camera",
        .author             = "Antmicro Ltd.",
        .methods            = &android::HalModule::moduleMethods,
        .dso                = NULL,
        .reserved           = {0}
    },
    .get_number_of_cameras  = android::HalModule::getNumberOfCameras,
    .get_camera_info        = android::HalModule::getCameraInfo,
    .set_callbacks          = android::HalModule::setCallbacks,
};
