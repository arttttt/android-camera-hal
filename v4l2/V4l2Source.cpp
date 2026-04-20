#include "V4l2Source.h"

#include <errno.h>
#include <utility>

#include <utils/Log.h>

#include "V4l2Device.h"
#include "V4l2CaptureThread.h"

#define LOG_TAG "Cam-V4l2Source"

namespace android {

V4l2Source::V4l2Source(V4l2Device *d)
    : dev(d),
      latest(nullptr),
      stopping(false) {}

V4l2Source::~V4l2Source() {
    stop();
}

bool V4l2Source::start() {
    if (thread && thread->isRunning()) return true;

    {
        std::lock_guard<std::mutex> lock(mutex);
        stopping = false;
        latest = nullptr;
        toRelease.clear();
    }

    thread.reset(new V4l2CaptureThread(this));
    if (!thread->start("CameraCapture")) {
        ALOGE("failed to start capture thread");
        thread.reset();
        return false;
    }
    return true;
}

void V4l2Source::stop() {
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (stopping && !(thread && thread->isRunning())) return;
        stopping = true;
    }
    frameReady.notify_all();
    if (thread) {
        thread->stop();  /* signals stopFd; threadLoop breaks out of poll */
        thread.reset();
    }
}

const V4l2Device::VBuffer* V4l2Source::acquireNextFrame() {
    std::unique_lock<std::mutex> lock(mutex);
    frameReady.wait(lock, [this] { return latest != nullptr || stopping; });
    if (stopping) return nullptr;
    const V4l2Device::VBuffer *f = latest;
    latest = nullptr;
    return f;
}

void V4l2Source::releaseFrame(const V4l2Device::VBuffer *f) {
    if (!f) return;
    std::lock_guard<std::mutex> lock(mutex);
    toRelease.push_back(f);
}

void V4l2Source::flushPendingReleases() {
    std::vector<const V4l2Device::VBuffer*> pending;
    {
        std::lock_guard<std::mutex> lock(mutex);
        pending.swap(toRelease);
    }
    /* Direct QBUF (bypassing V4l2Device::unlock's DMABUF-deferred path):
     * the caller guarantees the GPU has drained, so the kernel can
     * reuse the slot immediately. */
    for (const V4l2Device::VBuffer *buf : pending) {
        if (buf && !dev->queueBufferExt(buf->index)) {
            ALOGE("flushPendingReleases: QBUF slot %d failed, errno=%d",
                  buf->index, errno);
        }
    }
}

Resolution V4l2Source::resolution() const {
    return dev->resolution();
}

Resolution V4l2Source::sensorResolution() const {
    return dev->sensorResolution();
}

bool V4l2Source::isRunning() const {
    return thread && thread->isRunning();
}

} /* namespace android */
