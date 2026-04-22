#include "StatsWorker.h"

#include <errno.h>
#include <poll.h>
#include <string.h>

#include <utils/Log.h>

#define LOG_TAG "Cam-StatsWorker"

namespace android {

StatsWorker::StatsWorker()
    : pendingValid(false),
      latestSeq(0),
      latestValid(false) {
    memset(&pending, 0, sizeof(pending));
    memset(&latest,  0, sizeof(latest));
}

StatsWorker::~StatsWorker() {
    stop();
}

void StatsWorker::submit(const Job &job) {
    {
        std::lock_guard<std::mutex> lock(inLock);
        pending      = job;
        pendingValid = true;
    }
    jobEvent.notify();
}

bool StatsWorker::peek(IpaStats *out, uint32_t *sequence) {
    if (!out || !sequence) return false;
    std::lock_guard<std::mutex> lock(outLock);
    if (!latestValid) return false;
    *out      = latest;
    *sequence = latestSeq;
    return true;
}

void StatsWorker::reset() {
    {
        std::lock_guard<std::mutex> lock(inLock);
        pendingValid = false;
    }
    {
        std::lock_guard<std::mutex> lock(outLock);
        latestValid = false;
    }
}

void StatsWorker::threadLoop() {
    while (!stopRequested()) {
        struct pollfd p[2];
        p[0].fd = jobEvent.fd(); p[0].events = POLLIN; p[0].revents = 0;
        p[1].fd = stopFd();      p[1].events = POLLIN; p[1].revents = 0;

        int n = ::poll(p, 2, -1);
        if (n < 0) {
            if (errno == EINTR) continue;
            ALOGE("poll failed, errno=%d", errno);
            break;
        }
        if (p[1].revents & POLLIN) break;
        if (!(p[0].revents & POLLIN)) continue;
        jobEvent.drain();

        Job job;
        bool haveJob = false;
        {
            std::lock_guard<std::mutex> lock(inLock);
            if (pendingValid) {
                job          = pending;
                haveJob      = true;
                pendingValid = false;
            }
        }
        if (!haveJob) continue;

        /* NeonStatsEncoder is stateless, so back-to-back compute()
         * calls need no tear-down in between. */
        IpaStats stats;
        encoder.compute(job.bayer, job.width, job.height, job.pixFmt, &stats);

        {
            std::lock_guard<std::mutex> lock(outLock);
            latest      = stats;
            latestSeq   = job.sequence;
            latestValid = true;
        }
    }
}

} /* namespace android */
