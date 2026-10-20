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

#define LOG_TAG "Cam-Focuser"

#include "Focuser.h"

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/videodev2.h>
#include <utils/Log.h>

#include "MediaGraph.h"

namespace android {

bool Focuser::open(const char *videoNode) {
    if (fd >= 0)
        return true;
    const std::string path = MediaGraph::lensSubdevFor(videoNode);
    if (path.empty())
        return false;
    fd = ::open(path.c_str(), O_RDWR);
    if (fd < 0) {
        ALOGW("open %s: %s", path.c_str(), strerror(errno));
        return false;
    }
    ALOGI("opened %s fd=%d", path.c_str(), fd);
    return true;
}

void Focuser::close() {
    if (fd >= 0) {
        ::close(fd);
        fd = -1;
    }
}

bool Focuser::setPosition(int32_t position) {
    if (fd < 0) {
        ALOGW("setPosition(%d): focuser not open", position);
        return false;
    }
    struct v4l2_control ctrl;
    memset(&ctrl, 0, sizeof(ctrl));
    ctrl.id    = V4L2_CID_FOCUS_ABSOLUTE;
    ctrl.value = position;
    if (ioctl(fd, VIDIOC_S_CTRL, &ctrl) < 0) {
        ALOGW("setPosition(%d): %s (%d)", position, strerror(errno), errno);
        return false;
    }
    ALOGD("setPosition: %d -> fd=%d OK", position, fd);
    return true;
}

} /* namespace android */
