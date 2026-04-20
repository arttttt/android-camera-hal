#ifndef V4L2_BAYER_SOURCE_H
#define V4L2_BAYER_SOURCE_H

#include "Resolution.h"
#include "V4l2Device.h"

namespace android {

/* Abstract producer of Bayer frames for the HAL pipeline. The concrete
 * V4l2Source pulls frames from a V4L2 capture device on its own
 * thread; future implementations could serve a ZSL ring buffer, a
 * test pattern, or a playback file. Consumers see only this
 * interface. */
class BayerSource {
public:
    virtual ~BayerSource() {}

    /* Begin producing frames. Preconditions (e.g. V4L2 streaming on)
     * are the caller's responsibility. Safe to call after stop(). */
    virtual bool start() = 0;

    /* Stop producing, release any held buffers, join any worker
     * threads. Safe to call when not running. */
    virtual void stop() = 0;

    /* True iff start() has succeeded and stop() has not yet been
     * called. Lets the owner gate start() on idempotency. */
    virtual bool isRunning() const = 0;

    /* Block until the next Bayer frame is available or the source
     * stops. Returns null on stop. The caller owns the frame for the
     * duration of its use and must hand it back via releaseFrame()
     * before the source may reuse the underlying V4L2 slot. */
    virtual const V4l2Device::VBuffer* acquireNextFrame() = 0;

    /* Hand a previously-acquired frame back to the source. Safe to
     * call from any thread. Released buffers are not returned to the
     * underlying V4L2 device until flushPendingReleases() is called —
     * this avoids handing a slot back to the sensor while the GPU is
     * still reading from it. */
    virtual void releaseFrame(const V4l2Device::VBuffer*) = 0;

    /* Return all previously-released buffers to the V4L2 device.
     * The caller must guarantee the GPU (or any other consumer)
     * has finished reading from them — typically by calling
     * IspPipeline::waitForPreviousFrame() first. */
    virtual void flushPendingReleases() = 0;

    /* The source's output resolution (what it's currently capturing
     * at). For a V4L2 source, tracks V4l2Device::resolution(). */
    virtual Resolution resolution() const = 0;

    /* The raw sensor's native resolution, used to map crop regions
     * received in sensor coordinates back to V4L2 coordinates. */
    virtual Resolution sensorResolution() const = 0;
};

} /* namespace android */

#endif /* V4L2_BAYER_SOURCE_H */
