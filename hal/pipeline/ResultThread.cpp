#include "ResultThread.h"

#include <atomic>
#include <errno.h>
#include <poll.h>

#include <utils/Log.h>

#include "PipelineStage.h"
#include "InFlightTracker.h"
#include "BayerSource.h"

#undef  LOG_TAG
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

bool ResultThread::isCtxReady(const PipelineContext *ctx) const {
    /* Ready once every JPEG job posted for it has come back, whatever
     * the status: a job holds a raw pointer to the ctx, and dispatch
     * destroys it. Jobs are posted only for committed frames, and a
     * failed or cancelled one has none. */
    return ctx->jpegPending.load(std::memory_order_acquire) == 0;
}

void ResultThread::drainQueueIntoPending() {
    PipelineContext *ctx = nullptr;
    while (deps.queue->tryPop(ctx)) {
        if (ctx) pending.push_back(ctx);
    }
}

void ResultThread::dispatchReadyPrefix() {
    while (!pending.empty() && isCtxReady(pending.front())) {
        PipelineContext *ctx = pending.front();
        pending.pop_front();
        completeCtx(ctx);
    }
}

void ResultThread::threadLoop() {
    while (!stopRequested()) {
        struct pollfd pfds[3] = {
            { deps.queue->fd(),                POLLIN, 0 },
            { deps.jpegCompletionWake->fd(),   POLLIN, 0 },
            { stopFd(),                        POLLIN, 0 },
        };
        int n = ::poll(pfds, 3, -1);
        if (n < 0) {
            if (errno == EINTR) continue;
            ALOGE("poll failed, errno=%d", errno);
            break;
        }
        if (pfds[2].revents & POLLIN) break;

        if (pfds[0].revents & POLLIN) deps.queue->drain();
        if (pfds[1].revents & POLLIN) deps.jpegCompletionWake->drain();

        drainQueueIntoPending();
        dispatchReadyPrefix();
    }

    /* Stop drain: dispatch whatever is ready. JpegWorker is stopped
     * before us (Camera::stopWorkers), and its own stop drain encodes
     * every job left in its queue, so by now every jpegPending has
     * come down and the pending ctxs are ready. */
    drainQueueIntoPending();
    dispatchReadyPrefix();
    /* Anything still pending at this point: snapshot encode never
     * completed (e.g. JpegWorker errored). Cancel the ctx and dispatch
     * so framework buffers return; a committed one completes
     * normally. */
    while (!pending.empty()) {
        PipelineContext *ctx = pending.front();
        pending.pop_front();
        ctx->status.cancel();
        completeCtx(ctx);
    }
}

} /* namespace android */
