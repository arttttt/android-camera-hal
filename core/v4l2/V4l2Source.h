#ifndef V4L2_V4L2_SOURCE_H
#define V4L2_V4L2_SOURCE_H

#include <stdint.h>

#include <mutex>
#include <optional>
#include <vector>

#include "EventFd.h"
#include "V4l2Device.h"
#include "mochacamera/ports/FrameSource.h"

namespace android {

/* FrameSource backed by a V4L2 capture device. It has no thread of its
 * own: the capture engine polls the device and calls in, and only that
 * thread dequeues or queues a buffer, so V4l2Device's slots have a
 * single writer. Leases give slots back through returnSlot() from any
 * thread; the slot is queued again on the capture thread. */
class V4l2Source : public mocha::camera::FrameSource,
                   public mocha::camera::SlotReturn {
public:
    explicit V4l2Source(V4l2Device *dev);
    ~V4l2Source() override;

    V4l2Source(const V4l2Source&) = delete;
    V4l2Source& operator=(const V4l2Source&) = delete;

    bool start() override;
    void stop() override;
    bool isRunning() const override;

    int fd() const override;
    int returnedFd() const override { return returnedWake.fd(); }

    std::vector<mocha::camera::BayerLease> drain(short revents) override;
    bool watchFrameStarts() override;
    std::vector<mocha::camera::FrameStart> frameStarts() override;
    void requeueReturned() override;

    void returnSlot(int slot, uint32_t generation) override;

    mocha::camera::Size resolution()       const override;

private:
    /* vb2 reports POLLERR while the queue is not streaming or holds no
     * buffer, and keeps reporting it; the capture thread backs off this
     * long rather than spin. */
    static constexpr int kErrorBackoffMs = 20;

    V4l2Device *dev;
    EventFd     returnedWake;

    /* Guards generation, running and returned: leases return slots
     * from any thread. */
    mutable std::mutex mutex;
    uint32_t           generation;
    bool               running;
    std::vector<int>   returned;
};

} /* namespace android */

#endif /* V4L2_V4L2_SOURCE_H */
