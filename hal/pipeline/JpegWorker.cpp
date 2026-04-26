#include "JpegWorker.h"

#include <atomic>
#include <errno.h>
#include <poll.h>

#include <utils/Log.h>

#include "BufferProcessor.h"
#include "PipelineContext.h"

#define LOG_TAG "Cam-JpegWorker"

namespace android {

JpegWorker::JpegWorker(const Deps &d) : deps(d) {}

JpegWorker::~JpegWorker() {
    stop();
}

void JpegWorker::runJob(const Job &job) {
    if (!job.ctx) return;
    if (job.outputIndex >= job.ctx->request.outputBuffers.size()) return;
    if (job.outputIndex >= job.ctx->outputJpegSnapshots.size()) return;

    deps.bufferProcessor->finalizeBlobOutput(
        job.ctx->request.outputBuffers[job.outputIndex],
        job.ctx->request.settings,
        job.ctx->outputJpegSnapshots[job.outputIndex],
        job.ctx->request.frameNumber);

    /* Atomic decrement; the prev-value tells us whether this was the
     * last outstanding BLOB for the ctx. memory_order_acq_rel matches
     * the producer side's release store of the initial count. */
    int prev = job.ctx->jpegPending.fetch_sub(1, std::memory_order_acq_rel);
    if (prev == 1)
        deps.completionWake->notify();
}

void JpegWorker::threadLoop() {
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
            Job job{nullptr, 0};
            while (deps.queue->tryPop(job)) {
                runJob(job);
            }
        }
    }

    /* Stop drain: encode any leftover jobs so the corresponding ctxs in
     * ResultThread's deque can complete. Marking them errored upstream
     * is also fine, but encoding is cheap relative to a wedged
     * shutdown. */
    Job job{nullptr, 0};
    while (deps.queue->tryPop(job)) {
        runJob(job);
    }
}

} /* namespace android */
