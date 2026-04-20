#ifndef V4L2_V4L2_CAPTURE_THREAD_H
#define V4L2_V4L2_CAPTURE_THREAD_H

#include "ThreadBase.h"

namespace android {

class V4l2Source;

/* Runs the V4L2 capture loop: poll on the device fd plus a stop
 * eventfd, perform non-blocking DQBUF with drain-to-latest, and hand
 * the freshest frame to the owning V4l2Source. */
class V4l2CaptureThread : public ThreadBase {
public:
    explicit V4l2CaptureThread(V4l2Source *owner);
    ~V4l2CaptureThread() override;

protected:
    void threadLoop() override;

private:
    V4l2Source *owner;
};

} /* namespace android */

#endif /* V4L2_V4L2_CAPTURE_THREAD_H */
