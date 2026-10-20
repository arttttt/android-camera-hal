#include "V4l2CaptureThread.h"

#include <errno.h>
#include <poll.h>

#include <utils/Log.h>
#include <utils/Timers.h>

#include "V4l2Device.h"
#include "V4l2Source.h"

#undef  LOG_TAG
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

    /* Stall watch: no good frame for kStallMs means the sensor or VI
     * has stopped (a manual exposure is at most 1 s, and the driver
     * stretches a frame to ~0.85 s at most). The owner is told once
     * per stall; a frame arriving afterwards rearms it. */
    nsecs_t lastFrame = systemTime();
    bool    stallReported = false;
    nsecs_t prevGood  = 0;   /* dequeue time of the previous good frame */

    while (!stopRequested()) {
        /* Consumer-released buffers stay in owner->toRelease until the
         * request thread's CaptureStage flushes them via
         * BayerSource::flushPendingReleases() — at which point the
         * GPU has drained and it's safe to requeue. We never touch
         * V4l2Device::unlock here; all QBUF-back-to-sensor calls on
         * this thread come from the drain-to-latest loop below, whose
         * buffers have never been handed to the GPU. */
        const int sinceMs = (int)((systemTime() - lastFrame) / 1000000);
        if (sinceMs >= kStallMs) {
            if (!stallReported) {
                stallReported = true;
                ALOGE("no frame from the sensor for %d ms", sinceMs);
                if (owner->onStall) owner->onStall();
            }
        }
        const int timeoutMs = stallReported ? -1 : kStallMs - sinceMs;

        pfds[0].revents = 0;
        pfds[1].revents = 0;
        int n = ::poll(pfds, 2, timeoutMs);
        if (n < 0) {
            if (errno == EINTR) continue;
            ALOGE("poll failed, errno=%d", errno);
            break;
        }
        if (pfds[1].revents & POLLIN) break;
        if (n == 0) continue;   /* timeout: the stall check above */
        if (pfds[0].revents & (POLLERR | POLLHUP | POLLNVAL)) {
            /* vb2 reports POLLERR while the queue is not streaming or
             * holds no buffer; poll would return at once forever. Wait
             * on the stop fd alone for a moment instead of spinning;
             * the stall watch still runs. */
            struct pollfd stopOnly = { stopFd(), POLLIN, 0 };
            ::poll(&stopOnly, 1, kErrorBackoffMs);
            continue;
        }
        if (!(pfds[0].revents & POLLIN)) continue;

        /* DQBUF everything ready — non-blocking: the V4L2 fd said
         * ready. Drain-to-latest keeps only the newest good frame and
         * requeues older slots that piled up behind while the consumer
         * was slow. A slot the driver flagged V4L2_BUF_FLAG_ERROR holds
         * a damaged frame: it goes straight back and is never handed
         * on. */
        int id = -1;
        int dropped = 0;
        int damaged = 0;
        for (;;) {
            bool corrupt = false;
            int next = dev->dequeueBufferNonBlockingExt(&corrupt);
            if (next < 0) break;
            if (corrupt) {
                if (!dev->queueBufferExt(next)) {
                    ALOGE("requeue of damaged slot %d failed: errno=%d",
                          next, errno);
                }
                ++damaged;
                continue;
            }
            if (id >= 0) {
                if (!dev->queueBufferExt(id)) {
                    ALOGE("drain-to-latest: QBUF slot %d failed: errno=%d",
                          id, errno);
                }
                ++dropped;
            }
            id = next;
        }
        if (damaged)
            ALOGW("capture-thread: %d damaged frame(s) requeued", damaged);
        if (dropped)
            ALOGD("capture-thread: dropped %d stale frame(s)", dropped);
        if (id < 0) {
            /* Ready signalled but nothing good there (can happen on
             * STREAMOFF). Loop back to check stopFd. */
            continue;
        }
        lastFrame     = systemTime();
        stallReported = false;

        const V4l2Device::VBuffer *frame = dev->slotAt(id);
        if (!frame) {
            ALOGE("slotAt(%d) returned null", id);
            continue;
        }
        {
            /* The slot belongs to this thread until it is handed out
             * below. Frames dropped by drain-to-latest stretch the
             * interval, which only makes the exposure start earlier
             * than it was: the safe direction for the AF freshness
             * gate reading it. */
            V4l2Device::VBuffer *slot = const_cast<V4l2Device::VBuffer *>(frame);
            slot->dequeuedNs = lastFrame;
            slot->intervalNs = prevGood ? lastFrame - prevGood : 0;
            prevGood = lastFrame;
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
