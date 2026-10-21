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

#include "Camera3Device.h"

#define MAX_CAMERAS 8
#define MAX_VIDEO_NODES 16

extern camera_module_t HAL_MODULE_INFO_SYM;

namespace android {
namespace HalModule {

static Camera3Device *sCameras[MAX_CAMERAS];
static int sNumCameras = 0;

/* The cameras' ids as the framework names them ("0", "1", ...), and for
 * each camera the ids of the others: the conflicting_devices of its
 * camera_info. Built once, with the cameras; the framework keeps the
 * pointers. */
static char  sIds[MAX_CAMERAS][4];
static char *sConflicts[MAX_CAMERAS][MAX_CAMERAS];

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

    for (int i = 0; i < MAX_VIDEO_NODES && sNumCameras < MAX_CAMERAS; i++) {
        char path[32];
        snprintf(path, sizeof(path), "/dev/video%d", i);

        if (!isV4l2CaptureDevice(path))
            continue;

        /* Default: first camera = back, rest = front */
        int facing = (sNumCameras == 0) ? CAMERA_FACING_BACK : CAMERA_FACING_FRONT;

        ALOGI("Found V4L2 capture device: %s (facing=%s)",
              path, facing == CAMERA_FACING_BACK ? "back" : "front");

        sCameras[sNumCameras++] = new Camera3Device(path, facing);
    }

    /* One sensor path and one ISP: the cameras stream one at a time,
     * so each conflicts with every other. */
    for (int i = 0; i < sNumCameras; i++)
        snprintf(sIds[i], sizeof(sIds[i]), "%d", i);
    for (int i = 0; i < sNumCameras; i++) {
        int n = 0;
        for (int j = 0; j < sNumCameras; j++)
            if (j != i)
                sConflicts[i][n++] = sIds[j];
    }

    ALOGI("Enumerated %d camera(s)", sNumCameras);
}

static int getNumberOfCameras() {
    enumerateCameras();
    return sNumCameras;
}

/* A camera id as the framework sends it: the decimal index, nothing
 * after it. False for anything else or a camera that is not there. */
static bool parseCameraId(const char *name, int *cameraId) {
    if (name == NULL || *name == '\0')
        return false;
    errno = 0;
    char *end = NULL;
    const long id = strtol(name, &end, 10);
    if (errno || *end != '\0' || id < 0 || id >= sNumCameras)
        return false;
    *cameraId = (int)id;
    return true;
}

static int getCameraInfo(int cameraId, struct camera_info *info) {
    enumerateCameras();
    if (cameraId < 0 || cameraId >= sNumCameras) {
        ALOGE("%s: invalid camera ID (%d)", __FUNCTION__, cameraId);
        return -ENODEV;
    }
    const int res = sCameras[cameraId]->cameraInfo(info);
    if (res != NO_ERROR)
        return res;

    /* Module 2.4 arbitration: a camera takes the whole of what streams
     * (resource_cost 100), and lists the others as conflicting, so the
     * service never has two open at once. */
    info->resource_cost               = 100;
    info->conflicting_devices         = sNumCameras > 1 ? sConflicts[cameraId] : NULL;
    info->conflicting_devices_length  = (size_t)(sNumCameras - 1);
    return NO_ERROR;
}

/* No camera of this module has a flash unit (FLASH_INFO_AVAILABLE is
 * false in the static characteristics), which camera_common.h answers
 * with -ENOSYS, and only that. */
static int setTorchMode(const char *cameraIdName, bool /*enabled*/) {
    enumerateCameras();
    int cameraId;
    if (!parseCameraId(cameraIdName, &cameraId))
        return -EINVAL;
    return -ENOSYS;
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

    int cameraId;
    if (!parseCameraId(name, &cameraId)) {
        ALOGE("%s: invalid camera ID (%s)", __FUNCTION__, name);
        return -EINVAL;
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
        .module_api_version = CAMERA_MODULE_API_VERSION_2_4,
        .hal_api_version    = HARDWARE_HAL_API_VERSION,
        .id                 = CAMERA_HARDWARE_MODULE_ID,
        .name               = "V4l2 Camera",
        .author             = "Artem Bambalov",
        .methods            = &android::HalModule::moduleMethods,
        .dso                = NULL,
        .reserved           = {0}
    },
    .get_number_of_cameras  = android::HalModule::getNumberOfCameras,
    .get_camera_info        = android::HalModule::getCameraInfo,
    .set_callbacks          = android::HalModule::setCallbacks,
    .get_vendor_tag_ops     = NULL,
    .open_legacy            = NULL,
    .set_torch_mode         = android::HalModule::setTorchMode,
    .init                   = NULL,
};
