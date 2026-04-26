#include "ResultThread.h"

#include <errno.h>
#include <poll.h>

#include <hardware/camera3.h>
#include <utils/Log.h>

#include "PipelineStage.h"
#include "InFlightTracker.h"
#include "BayerSource.h"

#define LOG_TAG "Cam-ResultThread"

namespace android {

ResultThread::ResultThread(const Deps &d) : deps(d) {}

ResultThread::~ResultThread() {
    stop();
}

void ResultThread::completeCtx(PipelineContext *ctx) {
    deps.resultDispatch->process(*ctx);
    /* Bayer releases pended into BayerSource's deferred list during
     * dispatch; flush now so V4L2's ring gets the slot back before the
     * sensor runs dry. */
    if (deps.bayerSource) deps.bayerSource->flushPendingReleases();

    std::unique_ptr<PipelineContext> owned =
        deps.tracker->removeBySequence(ctx->sequence);
    (void)owned;
}

void ResultThread::threadLoop() {
    while (!stopRequested()) {
        struct pollfd pfds[2] = {
            { deps.queue->fd(), POLLIN, 0 },
            { stopFd(),         POLLIN, 0 },
        };
        int n = ::poll(pfds, 2, -1);
        if (n < 0) {
            if (errno == EINTR) continue;
            ALOGE("poll failed, errno=%d", errno);
            break;
        }
        if (pfds[1].revents & POLLIN) break;

        if (pfds[0].revents & POLLIN) {
            deps.queue->drain();
            PipelineContext *ctx = nullptr;
            while (deps.queue->tryPop(ctx)) {
                if (ctx) completeCtx(ctx);
            }
        }
    }

    /* Stop drain: dispatch any leftovers PipelineThread pushed before /
     * during the stop signal. PipelineThread's own stop drain marks
     * unfinished ctxs ERROR_REQUEST upstream so they reach us with
     * errorCode != 0; the safety guard below catches anything that
     * slipped through. */
    PipelineContext *ctx = nullptr;
    while (deps.queue->tryPop(ctx)) {
        if (!ctx) continue;
        if (ctx->errorCode == 0) ctx->errorCode = CAMERA3_MSG_ERROR_REQUEST;
        completeCtx(ctx);
    }
}

} /* namespace android */
