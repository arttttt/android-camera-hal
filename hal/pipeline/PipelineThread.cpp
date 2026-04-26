#include "PipelineThread.h"

#include <errno.h>
#include <poll.h>
#include <unistd.h>

#include <hardware/camera3.h>
#include <utils/Log.h>

#include "PipelineStage.h"
#include "BufferProcessor.h"

#define LOG_TAG "Cam-PipelineThread"

namespace {
/* Per-fence wait cap used when draining on stop. Matches the Camera3
 * flush() expectation (< 1000 ms) and keeps a GPU hang from blocking
 * closeDevice forever. */
constexpr int stopDrainTimeoutMs = 1000;
} /* namespace */

namespace android {

PipelineThread::PipelineThread(const Deps &d) : deps(d) {}

PipelineThread::~PipelineThread() {
    stop();
}

bool PipelineThread::reapFences(PipelineContext *ctx) {
    if (!ctx) return true;
    std::vector<int> stillPending;
    stillPending.reserve(ctx->pendingFenceFds.size());
    for (int fd : ctx->pendingFenceFds) {
        if (fd < 0) continue;
        struct pollfd p = { fd, POLLIN, 0 };
        int r = ::poll(&p, 1, 0);
        if (r > 0 && (p.revents & POLLIN)) {
            ::close(fd);
        } else {
            stillPending.push_back(fd);
        }
    }
    ctx->pendingFenceFds = std::move(stillPending);
    return ctx->pendingFenceFds.empty();
}

void PipelineThread::drainFences(PipelineContext *ctx, int timeoutMs) {
    if (!ctx) return;
    for (int fd : ctx->pendingFenceFds) {
        if (fd < 0) continue;
        struct pollfd p = { fd, POLLIN, 0 };
        ::poll(&p, 1, timeoutMs);
        ::close(fd);
    }
    ctx->pendingFenceFds.clear();
}

void PipelineThread::completeCtx(PipelineContext *ctx) {
    /* CPU finalize for outputs whose GPU half ran into a host-mapped
     * backend buffer (YUV NV12, BLOB JPEG snapshot). Locks the gralloc,
     * encodes / repacks, unlocks. Runs here so ResultDispatch on the
     * downstream thread can hand the frame to the framework as soon as
     * the gralloc is filled. */
    if (deps.bufferProcessor)
        deps.bufferProcessor->finalizeCpuOutputs(*ctx);

    /* Stats consumer runs between fence reap and result dispatch —
     * the submit's fence has signalled (so the GPU-written stats
     * buffer is coherent) and the frame has not yet been returned to
     * the framework. Null when the infrastructure build opted out of
     * stats (e.g. CPU/GLES fallback backend). */
    if (deps.statsProcess && !ctx->errorCode)
        deps.statsProcess->process(*ctx);

    /* Hand the ctx off to ResultThread which owns ResultDispatchStage,
     * BayerSource flushPendingReleases, and the InFlightTracker
     * removeBySequence that destroys the ctx. The queue is sized to
     * absorb any plausible burst from PipelineThread; on overflow we
     * log fatal because that means the upstream ordering invariant is
     * broken. */
    if (!deps.resultQueue->push(ctx)) {
        ALOGE("ResultQueue push failed for frame %u — ctx leaked",
              ctx->request.frameNumber);
    }
}

void PipelineThread::threadLoop() {
    /* Main scheduling loop. Poll set is rebuilt every iteration since
     * the oldest in-flight context's fence fds change as submits land
     * and complete. */
    while (!stopRequested()) {
        std::vector<struct pollfd> pfds;
        pfds.reserve(8);
        pfds.push_back({ deps.queue->fd(), POLLIN, 0 });
        pfds.push_back({ stopFd(),         POLLIN, 0 });

        /* FIFO by single-queue submit order — the oldest in-flight
         * ctx always completes first, so only its fds need to be
         * watched to know when the next ResultDispatch can fire. */
        if (!inFlight.empty()) {
            for (int fd : inFlight.front()->pendingFenceFds) {
                if (fd >= 0) pfds.push_back({ fd, POLLIN, 0 });
            }
        }

        int n = ::poll(pfds.data(), pfds.size(), -1);
        if (n < 0) {
            if (errno == EINTR) continue;
            ALOGE("poll failed, errno=%d", errno);
            break;
        }
        if (pfds[1].revents & POLLIN) break;

        /* Accept new submits up to the in-flight cap. Drain the queue
         * eventfd only after tryPop confirms "queue empty" — draining
         * up front when the cap is about to block pop means the
         * eventfd counter goes to zero while the queue still has
         * items, and the next poll waits forever even though work is
         * already queued. */
        while (inFlight.size() < deps.maxInFlight) {
            PipelineContext *ctx = nullptr;
            if (!deps.queue->tryPop(ctx)) {
                deps.queue->drain();
                break;
            }
            if (!ctx) continue;

            /* DemosaicBlitStage is not alwaysRun — a context that
             * already errored earlier (CaptureStage on null Bayer,
             * ApplySettings on bad metadata) skips straight to
             * dispatch. Its pendingFenceFds stay empty so the reap
             * pass below sees it as done immediately. */
            if (!ctx->errorCode || deps.demosaicBlit->alwaysRun()) {
                deps.demosaicBlit->process(*ctx);
            }
            inFlight.push_back(ctx);
        }

        /* Reap contexts in submission order. ResultDispatchStage is
         * alwaysRun=true, so even an errored ctx with empty
         * pendingFenceFds lands here and gets a ERROR_REQUEST
         * notification out to the framework. */
        while (!inFlight.empty()) {
            PipelineContext *head = inFlight.front();
            if (!reapFences(head)) break;
            completeCtx(head);
            inFlight.pop_front();
        }
    }

    /* Stop drain: blocking wait on each remaining ctx's fences with a
     * per-fence timeout, then run ResultDispatch to complete them as
     * errors (if not already set). Buffers flow back to the framework
     * with CAMERA3_MSG_ERROR_REQUEST rather than leak. */
    while (!inFlight.empty()) {
        PipelineContext *head = inFlight.front();
        drainFences(head, stopDrainTimeoutMs);
        if (head->errorCode == 0) head->errorCode = CAMERA3_MSG_ERROR_REQUEST;
        completeCtx(head);
        inFlight.pop_front();
    }
}

} /* namespace android */
