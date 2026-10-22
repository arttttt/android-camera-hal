#include "V4l2Source.h"

#include <errno.h>
#include <poll.h>
#include <unistd.h>

#include <utils/Log.h>

#include "V4l2Device.h"

#undef  LOG_TAG
#define LOG_TAG "Cam-V4l2Source"

namespace android {

using mocha::camera::BayerLease;

V4l2Source::V4l2Source(V4l2Device *d)
    : dev(d),
      generation(0),
      running(false) {}

V4l2Source::~V4l2Source() {
    stop();
}

bool V4l2Source::start() {
    std::lock_guard<std::mutex> lock(mutex);
    if (running) return true;
    running = true;
    ++generation;
    returned.clear();
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
    /* Back to V4L2 directly, so the next STREAMOFF sees a clean ring. */
    for (int index : residue) {
        if (!dev->queueBuffer(index))
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

std::vector<BayerLease> V4l2Source::drain(short revents) {
    std::vector<BayerLease> frames;
    if (revents & (POLLERR | POLLHUP | POLLNVAL)) {
        ::usleep(kErrorBackoffMs * 1000);
        return frames;
    }
    if (!(revents & POLLIN)) return frames;

    uint32_t gen;
    {
        std::lock_guard<std::mutex> lock(mutex);
        gen = generation;
    }

    /* DQBUF everything ready -- non-blocking: the fd said ready. A slot
     * the driver flagged V4L2_BUF_FLAG_ERROR holds a damaged frame: it
     * goes straight back and is never handed on (the driver's count has
     * it, so the frame numbers stay whole). */
    int damaged = 0;
    for (;;) {
        V4l2Device::Dequeued info;
        const int id = dev->dequeueBufferNonBlocking(&info);
        if (id < 0) break;
        if (info.corrupt) {
            if (!dev->queueBuffer(id))
                ALOGE("requeue of damaged slot %d failed: errno=%d", id, errno);
            ++damaged;
            continue;
        }
        const V4l2Device::VBuffer *slot = dev->slotAt(id);
        if (!slot) {
            ALOGE("slotAt(%d) returned null", id);
            continue;
        }
        mocha::camera::BayerFrame frame;
        frame.slot        = slot->index;
        frame.cpu         = slot->buf;
        frame.pixFmt      = slot->pixFmt;
        frame.sequence    = info.sequence;
        frame.timestampNs = info.timestampNs;
        frames.emplace_back(frame, gen, this);
    }
    if (damaged) ALOGW("%d damaged frame(s) requeued", damaged);
    return frames;
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
        if (!dev->queueBuffer(index))
            ALOGE("requeue: QBUF slot %d failed, errno=%d", index, errno);
    }
}

void V4l2Source::returnSlot(int slot, uint32_t gen) {
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (!running || gen != generation) return;
        returned.push_back(slot);
    }
    returnedWake.notify();
}

mocha::camera::Size V4l2Source::resolution() const {
    const Resolution r = dev->resolution();
    return mocha::camera::Size{r.width, r.height};
}

mocha::camera::Size V4l2Source::sensorResolution() const {
    const Resolution r = dev->sensorResolution();
    return mocha::camera::Size{r.width, r.height};
}

} /* namespace android */
