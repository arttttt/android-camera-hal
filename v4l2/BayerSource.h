#ifndef V4L2_BAYER_SOURCE_H
#define V4L2_BAYER_SOURCE_H

#include <stdint.h>

#include <optional>

#include "BayerLease.h"
#include "Resolution.h"

namespace android {

/* Producer of Bayer frames for the camera core, driven by the capture
 * engine's thread: the engine polls fd() and returnedFd() with its own
 * work and calls back here, so every V4L2 ioctl (DQBUF, QBUF) happens
 * on that one thread. A frame is handed out as a lease; the lease
 * brings its slot back when its last holder lets go, from any thread,
 * and the slot is requeued on the capture thread. */
class BayerSource {
public:
    virtual ~BayerSource() {}

    /* A new run. Preconditions (V4L2 streaming on) are the caller's.
     * Called while the capture engine is not running. */
    virtual bool start() = 0;

    /* End of the run: the slot held as latest and the slots returned
     * since the last requeue go back to V4L2; leases still out return
     * nothing (their run is over). Called while the capture engine is
     * not running. */
    virtual void stop() = 0;

    virtual bool isRunning() const = 0;

    /* The V4L2 node, for the capture engine to watch. */
    virtual int fd() const = 0;

    /* Readable when leases have returned slots since requeueReturned(). */
    virtual int returnedFd() const = 0;

    /* On the capture thread, with the revents of fd(): dequeue what is
     * ready and keep only the newest good frame, requeueing the rest.
     * True when a new frame is waiting for takeLatest(). */
    virtual bool drainToLatest(short revents) = 0;

    /* On the capture thread: the newest frame not yet taken, as a
     * lease; nothing if none came since the last take. */
    virtual std::optional<mocha::camera::BayerLease> takeLatest() = 0;

    /* On the capture thread: requeue the slots leases returned. */
    virtual void requeueReturned() = 0;

    /* The source's output resolution (what it is capturing at). */
    virtual Resolution resolution() const = 0;

    /* The sensor's native resolution, to map crop regions given in
     * sensor coordinates to capture coordinates. */
    virtual Resolution sensorResolution() const = 0;
};

} /* namespace android */

#endif /* V4L2_BAYER_SOURCE_H */
