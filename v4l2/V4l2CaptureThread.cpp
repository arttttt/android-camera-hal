#include "V4l2CaptureThread.h"

#include <errno.h>
#include <poll.h>

#include <utils/Log.h>

#include "V4l2Device.h"
#include "V4l2Source.h"

#define LOG_TAG "Cam-V4l2CaptureThread"

namespace android {

V4l2CaptureThread::V4l2CaptureThread(V4l2Source *o) : owner(o) {}

V4l2CaptureThread::~V4l2CaptureThread() {
    stop();
}

void V4l2CaptureThread::threadLoop() {
    V4l2Device *dev = owner->dev;

    struct pollfd pfds[2];
    pfds[0].fd = dev->fd();
    pfds[0].events = POLLIN;
    pfds[1].fd = stopFd();
    pfds[1].events = POLLIN;

    while (!stopRequested()) {
        /* Consumer-released buffers stay in owner->toRelease until the
         * request thread's CaptureStage flushes them via
         * BayerSource::flushPendingReleases() — at which point the
         * GPU has drained and it's safe to requeue. We never touch
         * V4l2Device::unlock here; all QBUF-back-to-sensor calls on
         * this thread come from the drain-to-latest loop below, whose
         * buffers have never been handed to the GPU. */
        pfds[0].revents = 0;
        pfds[1].revents = 0;
        int n = ::poll(pfds, 2, -1);
        if (n < 0) {
            if (errno == EINTR) continue;
            ALOGE("poll failed, errno=%d", errno);
            break;
        }
        if (pfds[1].revents & POLLIN) break;
        if (!(pfds[0].revents & POLLIN)) continue;

        /* First DQBUF — non-blocking: the V4L2 fd said ready. */
        int id = dev->dequeueBufferNonBlockingExt();
        if (id < 0) {
            /* ESPIPE-like: ready signalled but nothing there (can
             * happen on STREAMOFF). Loop back to check stopFd. */
            continue;
        }

        /* Drain-to-latest: requeue any older slots that piled up
         * behind while the consumer was slow. Leaves only the newest
         * frame in hand. */
        int dropped = 0;
        for (;;) {
            int next = dev->dequeueBufferNonBlockingExt();
            if (next < 0) break;
            if (!dev->queueBufferExt(id)) {
                ALOGE("drain-to-latest: QBUF slot %d failed: errno=%d",
                      id, errno);
            }
            id = next;
            ++dropped;
        }
        if (dropped)
            ALOGD("capture-thread: dropped %d stale frame(s)", dropped);

        const V4l2Device::VBuffer *frame = dev->slotAt(id);
        if (!frame) {
            ALOGE("slotAt(%d) returned null", id);
            continue;
        }

        /* Handoff: if the previous latest was never consumed, drop it
         * now (second level of drain-to-latest, at the source boundary
         * rather than the V4L2 done-queue). Use direct queueBufferExt
         * rather than dev->unlock — the buffer never reached the GPU
         * (consumer didn't acquire it, or we're stopping before the
         * pipeline saw it), so deferred-QBUF isn't needed and would
         * just pile up in V4l2Device::mPendingQbuf with nothing to
         * flush it. */
        {
            std::lock_guard<std::mutex> lock(owner->mutex);
            if (owner->stopping) {
                dev->queueBufferExt(frame->index);
                break;
            }
            if (owner->latest) {
                dev->queueBufferExt(owner->latest->index);
            }
            owner->latest = frame;
        }
        owner->frameReady.notify_one();
    }

    /* Exit path: requeue any held slots directly (no deferral) so
     * setStreaming(false) sees a clean ring. Everything in toRelease
     * and latest was already safe to return — consumer handed it
     * back, and drain-to-latest dropped candidates have never been
     * consumed. */
    std::vector<const V4l2Device::VBuffer*> residue;
    {
        std::lock_guard<std::mutex> lock(owner->mutex);
        residue.swap(owner->toRelease);
        if (owner->latest) {
            residue.push_back(owner->latest);
            owner->latest = nullptr;
        }
    }
    for (const V4l2Device::VBuffer *buf : residue) {
        if (buf) dev->queueBufferExt(buf->index);
    }
    owner->frameReady.notify_all();
}

} /* namespace android */
