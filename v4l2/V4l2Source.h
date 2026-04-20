#ifndef V4L2_V4L2_SOURCE_H
#define V4L2_V4L2_SOURCE_H

#include <condition_variable>
#include <memory>
#include <mutex>
#include <vector>

#include "BayerSource.h"

namespace android {

class V4l2Device;
class V4l2CaptureThread;

/* BayerSource implementation backed by V4L2. Owns a V4l2CaptureThread
 * that continuously dequeues the freshest frame and makes it available
 * to acquireNextFrame() via a condition-variable-guarded handoff.
 *
 * All V4L2 ioctls (DQBUF / QBUF) are performed on the capture thread
 * so V4l2Device's internal state (pending-QBUF list, buffer slots)
 * has a single writer. Consumers push buffers back via releaseFrame()
 * which queues them for the capture thread to return on its next
 * iteration. */
class V4l2Source : public BayerSource {
public:
    explicit V4l2Source(V4l2Device *dev);
    ~V4l2Source() override;

    V4l2Source(const V4l2Source&) = delete;
    V4l2Source& operator=(const V4l2Source&) = delete;

    bool start() override;
    void stop()  override;

    const V4l2Device::VBuffer* acquireNextFrame() override;
    void releaseFrame(const V4l2Device::VBuffer*) override;
    void flushPendingReleases() override;

    Resolution resolution()       const override;
    Resolution sensorResolution() const override;

    bool isRunning() const override;

private:
    friend class V4l2CaptureThread;

    V4l2Device                        *dev;
    std::unique_ptr<V4l2CaptureThread> thread;

    std::mutex                         mutex;
    std::condition_variable            frameReady;
    const V4l2Device::VBuffer         *latest;
    std::vector<const V4l2Device::VBuffer*> toRelease;
    bool                               stopping;
};

} /* namespace android */

#endif /* V4L2_V4L2_SOURCE_H */
