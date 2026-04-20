#include "RequestThread.h"

#include <errno.h>
#include <poll.h>

#include <utils/Log.h>

#include "Pipeline.h"
#include "InFlightTracker.h"

#define LOG_TAG "Cam-RequestThread"

namespace android {

RequestThread::RequestThread(EventQueue<PipelineContext*> *q,
                             Pipeline *p,
                             InFlightTracker *t)
    : queue(q),
      pipeline(p),
      tracker(t) {}

RequestThread::~RequestThread() {
    stop();
}

void RequestThread::threadLoop() {
    struct pollfd pfds[2];
    pfds[0].fd = queue->fd();
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

        queue->drain();

        PipelineContext *ctx = nullptr;
        while (queue->tryPop(ctx)) {
            if (!ctx) continue;
            pipeline->run(*ctx);
            /* Remove from tracker → destroys context at scope end. */
            std::unique_ptr<PipelineContext> owned =
                tracker->removeBySequence(ctx->sequence);
            (void)owned;
            if (stopRequested()) break;
        }
    }
}

} /* namespace android */
