#include "V4l2Source.h"

#include <errno.h>
#include <poll.h>
#include <unistd.h>

#include <utils/Log.h>
#include <utils/Timers.h>

#include "V4l2Device.h"

#undef  LOG_TAG
#define LOG_TAG "Cam-V4l2Source"

namespace android {

using mocha::camera::BayerLease;

V4l2Source::V4l2Source(V4l2Device *d)
    : dev(d),
      generation(0),
      running(false),
      latest(nullptr),
      prevGoodNs(0) {}

V4l2Source::~V4l2Source() {
    stop();
}

bool V4l2Source::start() {
    std::lock_guard<std::mutex> lock(mutex);
    if (running) return true;
    running = true;
    ++generation;
    returned.clear();
    latest     = nullptr;
    prevGoodNs = 0;
    returnedWake.drain();
    return true;
}

void V4l2Source::stop() {
    std::vector<int> residue;
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (!running) return;
        running = false;
        /* Leases still out belong to the run that ends here; what they
         * return later is ignored. */
        ++generation;
        residue.swap(returned);
    }
    if (latest) {
        residue.push_back(latest->index);
        latest = nullptr;
    }
    /* Back to V4L2 directly, so the next STREAMOFF sees a clean ring. */
    for (int index : residue) {
        if (!dev->queueBufferExt(index))
            ALOGE("stop: QBUF slot %d failed, errno=%d", index, errno);
    }
}

bool V4l2Source::isRunning() const {
    std::lock_guard<std::mutex> lock(mutex);
    return running;
}

int V4l2Source::fd() const {
    return dev->fd();
}

bool V4l2Source::drainToLatest(short revents) {
    if (revents & (POLLERR | POLLHUP | POLLNVAL)) {
        ::usleep(kErrorBackoffMs * 1000);
        return false;
    }
    if (!(revents & POLLIN)) return false;

    /* DQBUF everything ready -- non-blocking: the fd said ready.
     * Drain-to-latest keeps only the newest good frame and requeues
     * older slots that piled up behind while the consumer was slow. A
     * slot the driver flagged V4L2_BUF_FLAG_ERROR holds a damaged frame:
     * it goes straight back and is never handed on. */
    int id = -1;
    int dropped = 0;
    int damaged = 0;
    for (;;) {
        bool corrupt = false;
        const int next = dev->dequeueBufferNonBlockingExt(&corrupt);
        if (next < 0) break;
        if (corrupt) {
            if (!dev->queueBufferExt(next))
                ALOGE("requeue of damaged slot %d failed: errno=%d", next, errno);
            ++damaged;
            continue;
        }
        if (id >= 0) {
            if (!dev->queueBufferExt(id))
                ALOGE("drain-to-latest: QBUF slot %d failed: errno=%d", id, errno);
            ++dropped;
        }
        id = next;
    }
    if (damaged) ALOGW("%d damaged frame(s) requeued", damaged);
    if (dropped) ALOGD("dropped %d stale frame(s)", dropped);
    if (id < 0) return false;

    const V4l2Device::VBuffer *frame = dev->slotAt(id);
    if (!frame) {
        ALOGE("slotAt(%d) returned null", id);
        return false;
    }
    {
        /* The slot is this thread's until it is handed out. Frames
         * dropped above stretch the interval, which only makes the
         * exposure start earlier than it was: the safe direction for
         * the AF freshness gate reading it. */
        const nsecs_t now = systemTime();
        V4l2Device::VBuffer *slot = const_cast<V4l2Device::VBuffer *>(frame);
        slot->dequeuedNs = now;
        slot->intervalNs = prevGoodNs ? now - prevGoodNs : 0;
        prevGoodNs = now;
    }

    /* A previous latest nobody took is dropped now: drain-to-latest at
     * the source boundary rather than in the V4L2 done queue. It never
     * reached the GPU, so it goes straight back. */
    if (latest && !dev->queueBufferExt(latest->index))
        ALOGE("drain-to-latest: QBUF slot %d failed: errno=%d",
              latest->index, errno);
    latest = frame;
    return true;
}

std::optional<BayerLease> V4l2Source::takeLatest() {
    if (!latest) return std::nullopt;
    uint32_t gen;
    {
        std::lock_guard<std::mutex> lock(mutex);
        gen = generation;
    }
    BayerLease lease(latest, gen, this);
    latest = nullptr;
    return lease;
}

void V4l2Source::requeueReturned() {
    /* Drained before the list is taken: a slot returned from here on
     * wakes the next poll. */
    returnedWake.drain();
    std::vector<int> pending;
    {
        std::lock_guard<std::mutex> lock(mutex);
        pending.swap(returned);
    }
    for (int index : pending) {
        if (!dev->queueBufferExt(index))
            ALOGE("requeue: QBUF slot %d failed, errno=%d", index, errno);
    }
}

void V4l2Source::returnSlot(int index, uint32_t gen) {
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (!running || gen != generation) return;
        returned.push_back(index);
    }
    returnedWake.notify();
}

Resolution V4l2Source::resolution() const {
    return dev->resolution();
}

Resolution V4l2Source::sensorResolution() const {
    return dev->sensorResolution();
}

} /* namespace android */
