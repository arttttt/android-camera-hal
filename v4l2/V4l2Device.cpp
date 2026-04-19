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

#define LOG_TAG "Cam-V4l2Device"
#define LOG_NDEBUG NDEBUG

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <sys/mman.h>

#include <utils/Log.h>
#include <cstring>
#include <errno.h>
#include <cstdlib>
#include <utils/misc.h>
#include <utils/Vector.h>
#include <cassert>

#include "V4l2Device.h"

namespace android {

/******************************************************************************\
                                    Helpers
\******************************************************************************/

static inline int openFd(const char *path) {
    assert(path);
    int flags = O_RDWR;
#ifdef V4L2DEVICE_USE_POLL
    flags |= O_NONBLOCK;
#endif
    int fd = open(path, flags);
    ALOGV("open %s = %d", path, fd);
    return fd;
}

static inline void closeFd(int *fd) {
    assert(fd);
    close(*fd);
    ALOGV("close %d", *fd);
    *fd = -1;
}

/******************************************************************************\
                                   V4l2Device
\******************************************************************************/

/**
 * \class V4l2Device
 *
 * Simple wrapper for part of V4L2 camera interface.
 */

/**
 * Initializes object.
 *
 * \parameter devNode Path to V4L2 device node
 */
V4l2Device::V4l2Device(const char *devNode)
    : mFd(-1)
    , mConnected(false)
    , mStreaming(false)
    , mDevNode(strdup(devNode))
    , mPixelFormat(0)
    , mMemoryType(V4L2_MEMORY_MMAP)
    , mPendingQbufCount(0)
    , mFocuserFd(-1)
{
    memset(&mFormat, 0, sizeof(mFormat));
    for (int i = 0; i < V4L2DEVICE_BUF_COUNT; i++) {
        mDmaBufFds[i] = -1;
        mBuf[i].index = i;
        mPendingQbuf[i] = -1;
    }
    mPFd.fd = -1;
    mPFd.events = POLLIN | POLLRDNORM;

#if V4L2DEVICE_FPS_LIMIT > 0
    mLastTimestamp = 0;
#endif

#ifdef V4L2DEVICE_OPEN_ONCE
    connect();
#endif
}

V4l2Device::~V4l2Device() {
    if(isStreaming()) {
        iocStreamOff();
    }
    closeFocuser();
    cleanup();
    free((void *)mDevNode);
}

bool V4l2Device::openFocuser(const char *subdevPath) {
    if (mFocuserFd >= 0)
        return true;
    mFocuserFd = open(subdevPath, O_RDWR);
    if (mFocuserFd < 0) {
        ALOGW("openFocuser(%s): %s (%d)", subdevPath, strerror(errno), errno);
        return false;
    }
    ALOGD("Focuser opened: %s fd=%d", subdevPath, mFocuserFd);
    return true;
}

bool V4l2Device::setFocusPosition(int32_t position) {
    if (mFocuserFd < 0)
        return false;

    struct v4l2_control ctrl;
    memset(&ctrl, 0, sizeof(ctrl));
    ctrl.id = V4L2_CID_FOCUS_ABSOLUTE;
    ctrl.value = position;

    if (ioctl(mFocuserFd, VIDIOC_S_CTRL, &ctrl) < 0) {
        ALOGW("setFocusPosition(%d): %s (%d)", position, strerror(errno), errno);
        return false;
    }
    return true;
}

void V4l2Device::closeFocuser() {
    if (mFocuserFd >= 0) {
        close(mFocuserFd);
        mFocuserFd = -1;
    }
}

/**
 * Returns array of camera's supported resolutions.
 */
const Vector<V4l2Device::Resolution> & V4l2Device::availableResolutions() {
    if(!mAvailableResolutions.isEmpty()) {
        return mAvailableResolutions;
    }

    int fd;
    bool fdNeedsClose = false;
    Vector<V4l2Device::Resolution> formats;

    if(mFd >= 0) {
        fd = mFd;
    } else {
        fd = openFd(mDevNode);
        fdNeedsClose = true;
    }
    if(fd < 0) {
        ALOGE("Could not open %s: %s (%d)", mDevNode, strerror(errno), errno);
        return mAvailableResolutions;
    }

    if (!mPixelFormat)
        negotiatePixelFormat(fd);

    struct v4l2_frmsizeenum frmSize;
    memset(&frmSize, 0, sizeof(frmSize));
    frmSize.pixel_format = mPixelFormat;

    errno = 0;
    while(ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &frmSize) == 0) {
        ++frmSize.index;

        /* Skip duplicate resolutions (sensor may list same size at different FPS) */
        bool dup = false;
        for (size_t i = 0; i < formats.size(); i++) {
            if (formats[i].width == frmSize.discrete.width &&
                formats[i].height == frmSize.discrete.height) {
                dup = true;
                break;
            }
        }
        if (dup)
            continue;

        ALOGD("%s: Found resolution: %dx%d", mDevNode, frmSize.discrete.width, frmSize.discrete.height);
        formats.add();
        formats.editTop().width = frmSize.discrete.width;
        formats.editTop().height = frmSize.discrete.height;
    }
    if(errno && errno != EINVAL) {
        ALOGW("Get available formats: %s (%d)", strerror(errno), errno);
    }

    if(fdNeedsClose) {
        closeFd(&fd);
    }

    mAvailableResolutions = formats;
    return mAvailableResolutions;
}

/**
 * Returns V4l2Device::Resolution with highest possible width and highest
 * possible height. This might not to be valid camera resolution.
 */
V4l2Device::Resolution V4l2Device::sensorResolution() {
    const Vector<V4l2Device::Resolution> &formats = availableResolutions();
    V4l2Device::Resolution max = {0, 0};
    for(size_t i = 0; i < formats.size(); ++i) {
        if(formats[i].width > max.width)
            max.width = formats[i].width;
        if(formats[i].height > max.height)
            max.height = formats[i].height;
    }
    return max;
}

/**
 * Sets new resolution. The resolution must be supported by camera. If it does
 * not, false is returned. Call only with disabled streaming.
 */
bool V4l2Device::setResolution(unsigned width, unsigned height) {
    if(mFormat.fmt.pix.width == width && mFormat.fmt.pix.height == height)
        return true;

    ALOGD("New resolution: %dx%d", width, height);
    if(isConnected()) {
        bool wasStreaming = mStreaming;
        if(wasStreaming)
            iocStreamOff();

        for(int i = 0; i < V4L2DEVICE_BUF_COUNT; ++i)
            mBuf[i].unmap();

        if(!setResolutionAndAllocateBuffers(width, height)) {
            ALOGE("Could not change resolution to %dx%d", width, height);
            return false;
        }

        if(wasStreaming)
            iocStreamOn();

        return true;
    } else {
        mFormat.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        mFormat.fmt.pix.pixelformat = mPixelFormat;
        mFormat.fmt.pix.width = width;
        mFormat.fmt.pix.height = height;
        return true;
    }
}

/**
 * Returns current resolution
 */
V4l2Device::Resolution V4l2Device::resolution() {
    Resolution res;
    res.width = mFormat.fmt.pix.width;
    res.height = mFormat.fmt.pix.height;
    return res;
}

bool V4l2Device::negotiatePixelFormat(int fd) {
    /* Preferred formats in priority order */
    static const uint32_t preferred[] = {
        V4L2_PIX_FMT_UYVY,
        V4L2_PIX_FMT_YUYV,
        V4L2_PIX_FMT_SBGGR10,
        V4L2_PIX_FMT_SRGGB10,
        V4L2_PIX_FMT_SGRBG10,
        V4L2_PIX_FMT_SGBRG10,
        V4L2_PIX_FMT_SBGGR8,
        V4L2_PIX_FMT_SRGGB8,
        V4L2_PIX_FMT_SGRBG8,
        V4L2_PIX_FMT_SGBRG8,
    };

    /* Enumerate all formats the device supports */
    struct v4l2_fmtdesc fmtDesc;
    memset(&fmtDesc, 0, sizeof(fmtDesc));
    fmtDesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmtDesc.index = 0;

    uint32_t fallbackFormat = 0;
    while (ioctl(fd, VIDIOC_ENUM_FMT, &fmtDesc) == 0) {
        ALOGD("%s: format %d: %.4s (%s)", mDevNode, fmtDesc.index,
              (const char *)&fmtDesc.pixelformat, fmtDesc.description);

        /* Check if this format has any frame sizes */
        struct v4l2_frmsizeenum frmSize;
        memset(&frmSize, 0, sizeof(frmSize));
        frmSize.pixel_format = fmtDesc.pixelformat;
        bool hasFrameSizes = (ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &frmSize) == 0);

        if (!hasFrameSizes) {
            ALOGD("%s: format %.4s has no frame sizes, skipping", mDevNode,
                  (const char *)&fmtDesc.pixelformat);
            fmtDesc.index++;
            continue;
        }

        /* Check if this is a preferred format */
        for (size_t i = 0; i < sizeof(preferred) / sizeof(preferred[0]); i++) {
            if (fmtDesc.pixelformat == preferred[i]) {
                mPixelFormat = fmtDesc.pixelformat;
                ALOGI("%s: selected format %.4s", mDevNode,
                      (const char *)&mPixelFormat);
                return true;
            }
        }

        if (!fallbackFormat)
            fallbackFormat = fmtDesc.pixelformat;

        fmtDesc.index++;
    }

    /* No preferred format found — use first with frame sizes */
    if (fallbackFormat) {
        mPixelFormat = fallbackFormat;
        ALOGW("%s: no preferred format, using %.4s", mDevNode,
              (const char *)&mPixelFormat);
        return true;
    }

    ALOGE("%s: no usable formats available", mDevNode);
    return false;
}

/**
 * Connects to camera, allocates buffers, starts streaming
 */
bool V4l2Device::connect() {
    if(isConnected())
        return false;

    mFd = openFd(mDevNode);
    if(mFd < 0) {
        ALOGE("Could not open %s: %s (%d)", mDevNode, strerror(errno), errno);
        return false;
    }

    if (!mPixelFormat && !negotiatePixelFormat(mFd)) {
        ALOGE("Could not negotiate pixel format for %s", mDevNode);
        closeFd(&mFd);
        return false;
    }

    unsigned width;
    unsigned height;
    if(mFormat.type) {
        width = mFormat.fmt.pix.width;
        height = mFormat.fmt.pix.height;
    } else {
        auto resolutions = availableResolutions();
        if(resolutions.isEmpty()) {
            ALOGE("No available resolutions found, aborting");
            closeFd(&mFd);
            return false;
        }
        auto defaultRes = resolutions[0];
        width = resolutions[0].width;
        height = resolutions[0].height;
        ALOGD("Using default resolution: %dx%d", defaultRes.width, defaultRes.height);
    }
    if(!setResolutionAndAllocateBuffers(width, height)) {
        ALOGE("Could not set resolution");
        closeFd(&mFd);
        return false;
    }

    mPFd.fd = mFd;
    mPFd.revents = 0;
    mConnected = true;

    return true;
}

/**
 * Stops streaming and disconnects from camera device.
 */
bool V4l2Device::disconnect() {
    if(!isConnected())
        return false;

    setStreaming(false);
#ifndef V4L2DEVICE_OPEN_ONCE
    cleanup();
#endif

    return true;
}

bool V4l2Device::setStreaming(bool enable) {
    if(enable == mStreaming)
        return true;

    if(!isConnected())
        return !enable;

    if(enable) {
        if(!iocStreamOn()) {
            ALOGE("Could not start streaming: %s (%d)", strerror(errno), errno);
            return false;
        }
    } else {
#ifdef V4L2DEVICE_OPEN_ONCE
        return true;
#else
        if(!iocStreamOff()) {
            ALOGE("Could not stop streaming: %s (%d)", strerror(errno), errno);
            return false;
        }
#endif
    }

    mStreaming = enable;

    return true;
}

/**
 * Lock buffer and return pointer to it. After processing buffer must be
 * unlocked with V4l2Device::unlock().
 */
const V4l2Device::VBuffer * V4l2Device::readLock() {
    assert(isConnected());
    assert(isStreaming());

    /* Flush deferred-QBUFs before dequeueing this frame. In DMABUF mode the
     * previous frame's GPU dispatch may still be reading its input slot at
     * Camera::unlock() time; deferring QBUF to the next readLock() trades a
     * one-frame return delay for guaranteed no V4L2↔GPU write/read overlap
     * (by this point the ISP's WaitForFences has drained frame N-1). */
    for (int i = 0; i < mPendingQbufCount; i++) {
        int id = mPendingQbuf[i];
        mPendingQbuf[i] = -1;
        if (!queueBuffer(id))
            ALOGE("deferred QBUF of slot %d failed: %s (%d)", id, strerror(errno), errno);
    }
    mPendingQbufCount = 0;

    int id = 0;
    if((id = dequeueBuffer()) < 0) {
        ALOGE("Could not dequeue buffer: %s (%d)", strerror(errno), errno);
        return NULL;
    }
    auto buf = &mBuf[id];
    return buf;
}

/**
 * Unlocks previously locked buffer.
 */
bool V4l2Device::unlock(const VBuffer *buf) {
    if(!buf || buf->index < 0 || buf->index >= V4L2DEVICE_BUF_COUNT)
        return false;

    if (mMemoryType == V4L2_MEMORY_DMABUF) {
        /* Defer return; see note in readLock(). */
        assert(mPendingQbufCount < V4L2DEVICE_BUF_COUNT);
        mPendingQbuf[mPendingQbufCount++] = buf->index;
        return true;
    }

    if(!queueBuffer(buf->index)) {
        ALOGE("Could not queue buffer %d: %s (%d)", buf->index, strerror(errno), errno);
        return false;
    }
    return true;
}

/**
 * Returns buffer with specified ID to the kernel
 */
bool V4l2Device::queueBuffer(unsigned id) {
    assert(mFd >= 0);

    struct v4l2_buffer bufInfo;
    memset(&bufInfo, 0, sizeof(bufInfo));
    bufInfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    bufInfo.memory = mMemoryType;
    bufInfo.index = id;
    if (mMemoryType == V4L2_MEMORY_DMABUF)
        bufInfo.m.fd = mDmaBufFds[id];

    if(ioctl(mFd, VIDIOC_QBUF, &bufInfo) < 0)
        return false;

    return true;
}

void V4l2Device::setDmaBufFds(const int *fds, int count) {
    /* Close any fds we already own. */
    for (int i = 0; i < V4L2DEVICE_BUF_COUNT; i++) {
        if (mDmaBufFds[i] >= 0) {
            ::close(mDmaBufFds[i]);
            mDmaBufFds[i] = -1;
        }
    }

    if (fds == NULL || count <= 0) {
        mMemoryType = V4L2_MEMORY_MMAP;
        return;
    }
    assert(count == V4L2DEVICE_BUF_COUNT);
    mMemoryType = V4L2_MEMORY_DMABUF;
    /* Take ownership of the fds — we will close them on destroy / re-call. */
    for (int i = 0; i < count; i++) mDmaBufFds[i] = fds[i];
}

/**
 * Dequeues next available buffer and returns its ID.
 */
int V4l2Device::dequeueBuffer() {
    assert(mFd >= 0);

    struct v4l2_buffer bufInfo;

    memset(&bufInfo, 0, sizeof(bufInfo));
    bufInfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    bufInfo.memory = mMemoryType;
    bufInfo.index = 0;

#if V4L2DEVICE_FPS_LIMIT > 0
    auto timestamp = systemTime();
    nsecs_t extraTime = 1000000000LL / V4L2DEVICE_FPS_LIMIT - (timestamp - mLastTimestamp);
    if(extraTime / 1000 > 0)
        usleep((unsigned)(extraTime / 1000));
    mLastTimestamp = systemTime();
#endif

    do {
#ifdef V4L2DEVICE_USE_POLL
        if((errno = 0, poll(&mPFd, 1, 5000)) <= 0) {
            errno = ETIME;
            return -1;
        }
#endif
    } while((errno = 0, ioctl(mFd, VIDIOC_DQBUF, &bufInfo)) < 0 && (errno == EINVAL || errno == EAGAIN));
    if(errno)
        return -1;

    return (int)bufInfo.index;
}

bool V4l2Device::setControl(uint32_t id, int32_t value) {
    if(mFd < 0)
        return false;

    struct v4l2_control ctrl;
    memset(&ctrl, 0, sizeof(ctrl));
    ctrl.id = id;
    ctrl.value = value;

    if(ioctl(mFd, VIDIOC_S_CTRL, &ctrl) < 0) {
        ALOGE("setControl(0x%x, %d) FAILED: %s (%d)", id, value,
              strerror(errno), errno);
        return false;
    }
    ALOGD("setControl(0x%x, %d) OK", id, value);
    return true;
}

bool V4l2Device::queryControl(uint32_t id, int32_t *min, int32_t *max, int32_t *def) {
    if (mFd < 0)
        return false;

    struct v4l2_queryctrl qc;
    memset(&qc, 0, sizeof(qc));
    qc.id = id;
    if (ioctl(mFd, VIDIOC_QUERYCTRL, &qc) < 0) {
        ALOGW("queryControl(0x%x) FAILED: %s", id, strerror(errno));
        return false;
    }
    if (min) *min = qc.minimum;
    if (max) *max = qc.maximum;
    if (def) *def = qc.default_value;
    ALOGD("queryControl(0x%x): min=%d max=%d def=%d", id, qc.minimum, qc.maximum, qc.default_value);
    return true;
}

bool V4l2Device::iocStreamOff() {
    assert(mFd >= 0);
    assert(mFormat.type);

    errno = 0;
    unsigned type = mFormat.type;
    if(ioctl(mFd, VIDIOC_STREAMOFF, &type) == 0) {
        mStreaming = false;
    } else {
        ALOGV("%s: %s (%d)", __FUNCTION__, strerror(errno), errno);
    }
    return !errno;
}

bool V4l2Device::iocStreamOn() {
    assert(mFd >= 0);
    assert(mFormat.type);

    errno = 0;
    unsigned type = mFormat.type;
    if(ioctl(mFd, VIDIOC_STREAMON, &type) == 0) {
        mStreaming = true;
    } else {
        ALOGV("%s: %s (%d)", __FUNCTION__, strerror(errno), errno);
    }
    return !errno;
}

bool V4l2Device::iocSFmt(unsigned width, unsigned height) {
    assert(mFd >= 0);
    assert(!mStreaming);

    struct v4l2_format format;
    memset(&format, 0, sizeof(format));

    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.pixelformat = mPixelFormat;
    format.fmt.pix.width = width;
    format.fmt.pix.height = height;

    errno = 0;
    if(ioctl(mFd, VIDIOC_S_FMT, &format) == 0) {
        mFormat = format;
        ALOGD("S_FMT: %ux%u pixfmt=0x%08x bytesperline=%u sizeimage=%u",
              format.fmt.pix.width, format.fmt.pix.height,
              format.fmt.pix.pixelformat,
              format.fmt.pix.bytesperline, format.fmt.pix.sizeimage);
    } else {
        ALOGV("%s(w=%u, h=%u): %s (%d)", __FUNCTION__, width, height, strerror(errno), errno);
    }

    return !errno;
}

bool V4l2Device::iocReqBufs(unsigned *count) {
    assert(mFd >= 0);
    assert(count);

    struct v4l2_requestbuffers bufRequest;
    memset(&bufRequest, 0, sizeof(bufRequest));

    bufRequest.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    bufRequest.memory = mMemoryType;
    bufRequest.count = *count;

    errno = 0;
    if(ioctl(mFd, VIDIOC_REQBUFS, &bufRequest) == 0) {
        *count = bufRequest.count;
    } else {
        ALOGV("%s(count=%u): %s (%d)", __FUNCTION__, *count, strerror(errno), errno);
    }

    return !errno;
}

void V4l2Device::probeBufferModes() {
    assert(mFd >= 0);
    struct { const char *name; uint32_t mem; } modes[] = {
        { "DMABUF",  V4L2_MEMORY_DMABUF  },
        { "USERPTR", V4L2_MEMORY_USERPTR },
    };
    for (size_t i = 0; i < sizeof(modes) / sizeof(modes[0]); i++) {
        struct v4l2_requestbuffers rb;
        memset(&rb, 0, sizeof(rb));
        rb.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        rb.memory = modes[i].mem;
        rb.count  = 1;
        errno = 0;
        int ret = ioctl(mFd, VIDIOC_REQBUFS, &rb);
        ALOGD("probe: REQBUFS %-7s count=1 → ret=%d allocated=%u errno=%d (%s)",
              modes[i].name, ret, rb.count, errno, strerror(errno));
        if (ret == 0) {
            memset(&rb, 0, sizeof(rb));
            rb.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            rb.memory = modes[i].mem;
            rb.count  = 0;
            ioctl(mFd, VIDIOC_REQBUFS, &rb);
        }
    }
}

bool V4l2Device::iocReleaseBufs() {
    assert(mFd >= 0);

    struct v4l2_requestbuffers bufRequest;
    memset(&bufRequest, 0, sizeof(bufRequest));
    bufRequest.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    bufRequest.memory = mMemoryType;
    bufRequest.count = 0;

    errno = 0;
    if(ioctl(mFd, VIDIOC_REQBUFS, &bufRequest) != 0) {
        ALOGV("%s: %s (%d)", __FUNCTION__, strerror(errno), errno);
    }
    return !errno;
}

bool V4l2Device::iocQueryBuf(unsigned id, unsigned *offset, unsigned *len) {
    assert(mFd >= 0);
    assert(offset);
    assert(len);

    struct v4l2_buffer bufInfo;
    memset(&bufInfo, 0, sizeof(bufInfo));

    bufInfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    bufInfo.memory = V4L2_MEMORY_MMAP;
    bufInfo.index = id;

    errno = 0;
    if(ioctl(mFd, VIDIOC_QUERYBUF, &bufInfo) == 0) {
        *offset = bufInfo.m.offset;
        *len = bufInfo.length;
    } else {
        ALOGV("%s(id=%u): %s (%d)", __FUNCTION__, id, strerror(errno), errno);
    }

    return !errno;
}

bool V4l2Device::setResolutionAndAllocateBuffers(unsigned width, unsigned height) {
    assert(!mStreaming);

    for(int i = 0; i < V4L2DEVICE_BUF_COUNT; ++i) {
        mBuf[i].unmap();
    }

    /* Release kernel-side buffers before S_FMT — vb2 returns EBUSY otherwise */
    iocReleaseBufs();

    if(!iocSFmt(width, height)) {
        ALOGE("Could not set pixel format to %dx%d: %s (%d)", width, height, strerror(errno), errno);
        return false;
    }

    probeBufferModes();

    unsigned bufCount = V4L2DEVICE_BUF_COUNT;
    if(!iocReqBufs(&bufCount)) {
        ALOGE("Could not request buffer: %s (%d)", strerror(errno), errno);
        return false;
    }

    for(unsigned i = 0; i < bufCount; ++i) {
        /* DMABUF mode: the caller (Vulkan ISP) owns the backing memory; we
         * only bind the fd to queue slot i. No mmap, no per-buffer length. */
        if (mMemoryType == V4L2_MEMORY_DMABUF) {
            mBuf[i].buf    = NULL;
            mBuf[i].len    = 0;
            mBuf[i].pixFmt = mPixelFormat;
            mBuf[i].index  = i;
        } else {
            unsigned offset;
            unsigned len;
            if(!iocQueryBuf(i, &offset, &len)) {
                ALOGE("Could not query buffer %d: %s (%d)", i, strerror(errno), errno);
                return false;
            }
            if(!mBuf[i].map(mFd, offset, len, mPixelFormat)) {
                ALOGE("Could not allocate buffer %d (len = %d): %s (%d)", i, len, strerror(errno), errno);
                while(i--) mBuf[i].unmap();
                return false;
            }
            mBuf[i].index = i;
        }

        if(!queueBuffer(i)) {
            ALOGE("Could not queue buffer: %s (%d)", strerror(errno), errno);
            do mBuf[i].unmap(); while(i--);
            return false;
        }
    }

    return true;
}

void V4l2Device::cleanup() {
    for(int i = 0; i < V4L2DEVICE_BUF_COUNT; ++i) {
        mBuf[i].unmap();
    }

    /* Release ownership of any imported dma-buf fds. */
    setDmaBufFds(NULL, 0);

    closeFd(&mFd);
    mPFd.fd = -1;
    mConnected = false;
}

/******************************************************************************\
                              V4l2Device::VBuffer
\******************************************************************************/

/**
 * \class V4l2Device::VBuffer
 *
 * Video buffer abstraction.
 */

V4l2Device::VBuffer::~VBuffer() {
    if(buf) {
        ALOGD("V4l2Device::VBuffer: Memory leak!");
        abort();
    }
}

bool V4l2Device::VBuffer::map(int fd, unsigned offset, unsigned len, uint32_t pixelFormat) {
    assert(!this->buf);

    errno = 0;
    this->buf = (uint8_t*)mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, offset);
    if(this->buf == MAP_FAILED) {
        return false;
    }
    memset(this->buf, 0, len);
    this->len = len;
    this->pixFmt = pixelFormat;

    return true;
}

void V4l2Device::VBuffer::unmap() {
    if(buf) {
        munmap(buf, len);
        buf         = NULL;
        len         = 0;
    }
}

}; /* namespace android */
