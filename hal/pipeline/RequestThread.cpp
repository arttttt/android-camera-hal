#include "RequestThread.h"

#include <errno.h>
#include <poll.h>

#include <utils/Log.h>

#include "Pipeline.h"
#include "InFlightTracker.h"

#define LOG_TAG "Cam-RequestThread"

namespace android {

RequestThread::RequestThread(EventQueue<PipelineContext*> *in,
                             Pipeline *p,
                             EventQueue<PipelineContext*> *out,
                             InFlightTracker *t)
    : inQueue(in),
      pipeline(p),
      outQueue(out),
      tracker(t) {}

RequestThread::~RequestThread() {
    stop();
}

void RequestThread::threadLoop() {
    struct pollfd pfds[2];
    pfds[0].fd = inQueue->fd();
    pfds[0].events = POLLIN;
    pfds[1].fd = stopFd();
    pfds[1].events = POLLIN;

    while (!stopRequested()) {
        pfds[0].revents = 0;
        pfds[1].revents = 0;
        int n = ::poll(pfds, 2, -1);
        if (n < 0) {
            if (errno == EINTR) continue;
            ALOGE("poll failed, errno=%d", errno);
            break;
        }
        if (pfds[1].revents & POLLIN) break;

        inQueue->drain();

        PipelineContext *ctx = nullptr;
        while (inQueue->tryPop(ctx)) {
            if (!ctx) continue;
            pipeline->run(*ctx);
            /* Hand off to PipelineThread. Blocking push applies
             * backpressure when the downstream has hit maxInFlight; on
             * stop, requestStop() on the downstream queue wakes us and
             * we leave ctx in the tracker for closeDevice's drainAll
             * to error-complete. */
            if (!outQueue->pushBlocking(ctx)) break;
            if (stopRequested()) break;
        }
    }
}

} /* namespace android */
