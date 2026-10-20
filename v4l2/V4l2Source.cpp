#include "V4l2Source.h"

#include <errno.h>
#include <utility>

#include <utils/Log.h>

#include "V4l2Device.h"
#include "V4l2CaptureThread.h"

#undef  LOG_TAG
#define LOG_TAG "Cam-V4l2Source"

namespace android {

V4l2Source::V4l2Source(V4l2Device *d)
    : dev(d),
      latest(nullptr),
      stopping(false) {
    for (int &r : refs) r = 0;
}

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
        for (int &r : refs) r = 0;
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

const V4l2Device::VBuffer* V4l2Source::acquireNextFrame(
        const std::function<bool()> &abortCheck) {
    std::unique_lock<std::mutex> lock(mutex);
    frameReady.wait(lock, [this, &abortCheck] {
        return latest != nullptr || stopping || (abortCheck && abortCheck());
    });
    if (stopping) return nullptr;
    if (abortCheck && abortCheck()) return nullptr;
    const V4l2Device::VBuffer *f = latest;
    latest = nullptr;
    if (f && f->index >= 0 && f->index < V4L2DEVICE_BUF_COUNT)
        refs[f->index] = 1;
    return f;
}

void V4l2Source::notifyForAbort() {
    /* No state to flip — the caller has already set whatever flag the
     * abortCheck callback inspects (e.g. ctx.status). All we need
     * is to wake the cv so blocked waiters re-evaluate the predicate. */
    frameReady.notify_all();
}

void V4l2Source::releaseFrame(const V4l2Device::VBuffer *f) {
    if (!f) return;
    std::lock_guard<std::mutex> lock(mutex);
    if (f->index >= 0 && f->index < V4L2DEVICE_BUF_COUNT) {
        /* A release with no reference left is a stale one from before
         * the source restarted (refs were reset); the slot was
         * requeued by the restart already. */
        if (refs[f->index] <= 0) return;
        if (--refs[f->index] > 0) return;
    }
    toRelease.push_back(f);
}

void V4l2Source::retainFrame(const V4l2Device::VBuffer *f) {
    if (!f || f->index < 0 || f->index >= V4L2DEVICE_BUF_COUNT) return;
    std::lock_guard<std::mutex> lock(mutex);
    ++refs[f->index];
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
