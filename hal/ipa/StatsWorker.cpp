#include "StatsWorker.h"

#include <errno.h>
#include <poll.h>
#include <string.h>

#include <utils/Log.h>

#define LOG_TAG "Cam-StatsWorker"

namespace android {

StatsWorker::StatsWorker()
    : blackLevel(0),
      pendingValid(false),
      phase(0),
      latestSeq(0),
      latestValid(false) {
    memset(&pending,    0, sizeof(pending));
    memset(&currentJob, 0, sizeof(currentJob));
    memset(&latest,     0, sizeof(latest));
    NeonStatsEncoder::resetPartial(&partial);
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
    /* Cycle state is owned by the worker thread alone, but a race
     * with an in-flight phase is benign: the next finalize would just
     * publish into the freshly-invalidated latest slot, and the peek
     * caller would see valid stats again. When the worker is stopped
     * around reset() the cycle state is static and this clears it. */
    phase = 0;
    NeonStatsEncoder::resetPartial(&partial);
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

        /* Idle → start a new cycle on the most recent pending Bayer.
         * Each incoming submit advances one phase; phaseCount phases
         * complete and publish one IpaStats. */
        if (phase == 0) {
            Job j;
            {
                std::lock_guard<std::mutex> lock(inLock);
                if (!pendingValid) continue;
                j = pending;
                pendingValid = false;
            }
            currentJob = j;
            NeonStatsEncoder::resetPartial(&partial);
        }

        const int rowsPerPhase = IpaStats::PATCH_Y / StatsWorker::phaseCount;
        const int pyStart      = phase * rowsPerPhase;
        const int pyEnd        = (phase + 1 == StatsWorker::phaseCount)
                                    ? IpaStats::PATCH_Y
                                    : (phase + 1) * rowsPerPhase;

        encoder.computeRange(currentJob.bayer,
                             currentJob.width, currentJob.height,
                             currentJob.pixFmt, blackLevel,
                             &partial, pyStart, pyEnd);

        ++phase;

        if (phase >= StatsWorker::phaseCount) {
            IpaStats result;
            NeonStatsEncoder::finalize(partial, currentJob.pixFmt,
                                        blackLevel, &result);
            {
                std::lock_guard<std::mutex> lock(outLock);
                latest      = result;
                latestSeq   = currentJob.sequence;
                latestValid = true;
            }
            phase = 0;
        }
    }
}

} /* namespace android */
