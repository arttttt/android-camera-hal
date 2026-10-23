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

#define LOG_TAG "Cam-SensorSubdev"

#include "SensorSubdev.h"

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/v4l2-subdev.h>
#include <utils/Log.h>

namespace android {

using mocha::camera::Rect;
using mocha::camera::Size;

namespace {

/* The sensor's source pad, the one its frames leave by. */
constexpr uint32_t kSourcePad = 0;

std::optional<Rect> selection(int fd, uint32_t which, uint32_t target) {
    struct v4l2_subdev_selection sel;
    memset(&sel, 0, sizeof(sel));
    sel.which  = which;
    sel.pad    = kSourcePad;
    sel.target = target;
    if (ioctl(fd, VIDIOC_SUBDEV_G_SELECTION, &sel) < 0)
        return std::nullopt;
    return Rect{sel.r.left, sel.r.top, (int32_t)sel.r.width, (int32_t)sel.r.height};
}

} /* namespace */

std::optional<SensorGeometry> readSensorGeometry(const std::string &subdevNode,
                                                 const std::vector<Size> &outputs) {
    if (subdevNode.empty())
        return std::nullopt;
    const int fd = open(subdevNode.c_str(), O_RDWR);
    if (fd < 0) {
        ALOGW("%s: %s", subdevNode.c_str(), strerror(errno));
        return std::nullopt;
    }

    const std::optional<Rect> native =
        selection(fd, V4L2_SUBDEV_FORMAT_ACTIVE, V4L2_SEL_TGT_NATIVE_SIZE);
    if (!native || native->empty()) {
        ALOGW("%s: no pixel array reported: %s", subdevNode.c_str(), strerror(errno));
        close(fd);
        return std::nullopt;
    }

    /* A TRY format needs a media bus code; the active one is the
     * sensor's. */
    struct v4l2_subdev_format active;
    memset(&active, 0, sizeof(active));
    active.which = V4L2_SUBDEV_FORMAT_ACTIVE;
    active.pad   = kSourcePad;
    if (ioctl(fd, VIDIOC_SUBDEV_G_FMT, &active) < 0) {
        ALOGW("%s: G_FMT: %s", subdevNode.c_str(), strerror(errno));
        close(fd);
        return std::nullopt;
    }

    std::vector<SensorMode> modes;
    for (const Size &out : outputs) {
        struct v4l2_subdev_format f = active;
        f.which         = V4L2_SUBDEV_FORMAT_TRY;
        f.format.width  = out.width;
        f.format.height = out.height;
        if (ioctl(fd, VIDIOC_SUBDEV_S_FMT, &f) < 0 ||
            f.format.width != out.width || f.format.height != out.height) {
            ALOGW("%s: no mode for %ux%u", subdevNode.c_str(), out.width, out.height);
            continue;
        }
        const std::optional<Rect> window =
            selection(fd, V4L2_SUBDEV_FORMAT_TRY, V4L2_SEL_TGT_CROP);
        if (!window || window->empty()) {
            ALOGW("%s: no window reported for %ux%u", subdevNode.c_str(),
                  out.width, out.height);
            continue;
        }
        ALOGI("%s: %ux%u shows (%d,%d) %dx%d of %dx%d", subdevNode.c_str(),
              out.width, out.height, window->left, window->top,
              window->width, window->height, native->width, native->height);
        modes.push_back(SensorMode{out, *window});
    }
    close(fd);

    return SensorGeometry::fromModes(
        Size{(uint32_t)native->width, (uint32_t)native->height}, std::move(modes));
}

} /* namespace android */
