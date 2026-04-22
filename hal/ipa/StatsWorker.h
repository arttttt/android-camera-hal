#ifndef HAL_IPA_STATS_WORKER_H
#define HAL_IPA_STATS_WORKER_H

#include <stdint.h>

#include <mutex>

#include "EventFd.h"
#include "IpaStats.h"
#include "NeonStatsEncoder.h"
#include "ThreadBase.h"

namespace android {

/* Dedicated worker thread that reduces raw Bayer into IpaStats via
 * NeonStatsEncoder. Runs outside the GPU-submit critical path so the
 * CPU stats cost overlaps the Vulkan demosaic + blit submit instead
 * of serialising with it.
 *
 * Producer (RequestThread's StatsDispatchStage): submit(job) with the
 * latest Bayer pointer and sequence. Submits are latest-wins — any
 * pending job that has not started is discarded in favour of the new
 * one, because a stale Bayer is never more relevant to AE / AF / AWB
 * convergence than a newer one.
 *
 * Consumer (PipelineThread's StatsProcessStage): peek(&stats) returns
 * a snapshot of the most recently published IpaStats under mutex. One
 * frame of lag between submit and consumer peek is expected in steady
 * state; the IPA treats stats as an input, not a blocker.
 *
 * The raw Bayer pointer inside the job must stay valid for the
 * worker's NEON pass. In our pipeline the V4L2 slot is held by the
 * in-flight PipelineContext until ResultDispatch releases it, giving
 * the worker a generous window (sensor cadence × buffer-ring depth)
 * before the slot can be rewritten. */
class StatsWorker : public ThreadBase {
public:
    struct Job {
        const void *bayer;
        unsigned    width;
        unsigned    height;
        uint32_t    pixFmt;
        uint32_t    sequence;
    };

    StatsWorker();
    ~StatsWorker() override;

    /* Post the latest work to the worker. Overwrites any pending but
     * not-yet-started job. Safe from any thread. */
    void submit(const Job &job);

    /* Copy the latest published IpaStats into `out` and its source
     * frame sequence into `*sequence`. Returns false on cold start
     * (no publication yet). Safe from any thread. */
    bool peek(IpaStats *out, uint32_t *sequence);

    /* Drop the pending job and the latest publication. Call between
     * sessions so stats from the previous capture do not leak into
     * the next one. Safe while the worker runs — an in-flight compute
     * may still publish a result that a subsequent peek would see. */
    void reset();

protected:
    void threadLoop() override;

private:
    NeonStatsEncoder encoder;

    EventFd    jobEvent;
    std::mutex inLock;
    Job        pending;
    bool       pendingValid;

    std::mutex outLock;
    IpaStats   latest;
    uint32_t   latestSeq;
    bool       latestValid;
};

} /* namespace android */

#endif /* HAL_IPA_STATS_WORKER_H */
