#include "ResultThread.h"

#include <atomic>
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

bool ResultThread::isCtxReady(const PipelineContext *ctx) const {
    /* Errored ctxs dispatch immediately as ERROR_REQUEST regardless of
     * pending JPEG work — JpegWorker's queue won't carry their jobs
     * because PipelineThread releases the snapshots locally on error. */
    if (ctx->errorCode) return true;
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

    /* Stop drain: wait for pending ctxs to become ready (best-effort)
     * then dispatch. JpegWorker is stopped after us in the lifecycle
     * (since we depend on it for jpegPending decrements), so any
     * pending ctxs here that are still parked on jpegPending will be
     * unblocked by JpegWorker's own stop drain that processes its
     * remaining jobs before exiting. */
    drainQueueIntoPending();
    dispatchReadyPrefix();
    /* Anything still pending at this point: snapshot encode never
     * completed (e.g. JpegWorker errored). Mark the ctx as
     * ERROR_REQUEST and dispatch so framework buffers return. */
    while (!pending.empty()) {
        PipelineContext *ctx = pending.front();
        pending.pop_front();
        if (ctx->errorCode == 0) ctx->errorCode = CAMERA3_MSG_ERROR_REQUEST;
        completeCtx(ctx);
    }
}

} /* namespace android */
