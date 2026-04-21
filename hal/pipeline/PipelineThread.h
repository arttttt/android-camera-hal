#ifndef HAL_PIPELINE_PIPELINE_THREAD_H
#define HAL_PIPELINE_PIPELINE_THREAD_H

#include <cstddef>
#include <deque>

#include "ThreadBase.h"
#include "EventQueue.h"
#include "PipelineContext.h"

namespace android {

class PipelineStage;
class BayerSource;
class InFlightTracker;

/* GPU-submit side of the async pipeline.
 *
 * Consumes PipelineContexts that RequestThread hands off after the
 * capture stages, runs DemosaicBlit to kick off the GPU submit, then
 * drives the submit's sync_fd in a poll() set so ResultDispatch runs
 * the instant the fence signals — with no vkWaitForFences on the hot
 * path. Up to maxInFlight contexts may overlap; RequestThread blocks
 * on pushBlocking when the queue is full.
 *
 * On stop: waits up to ~1s per pending fence, then runs ResultDispatch
 * for each so framework-owned buffers return cleanly and the tracker
 * is emptied. Matches the build-once / survive-close lifetime — the
 * thread object survives closeDevice → openDevice cycles. */
class PipelineThread : public ThreadBase {
public:
    struct Deps {
        EventQueue<PipelineContext*> *queue;
        PipelineStage                *demosaicBlit;
        PipelineStage                *resultDispatch;
        BayerSource                  *bayerSource;
        InFlightTracker              *tracker;
        std::size_t                   maxInFlight;
    };

    explicit PipelineThread(const Deps &deps);
    ~PipelineThread() override;

protected:
    void threadLoop() override;

private:
    /* Non-blocking sweep: for each sync_fd in ctx.pendingFenceFds,
     * poll(timeout=0); close + remove if signalled. Returns true when
     * the vector drained to empty. */
    bool reapFences(PipelineContext *ctx);

    /* Blocking drain: wait up to timeoutMs on remaining pendingFenceFds
     * and close all of them regardless. Used during stop. */
    void drainFences(PipelineContext *ctx, int timeoutMs);

    /* Dispatch ctx's terminal stage + cleanup: flush pending Bayer
     * releases and remove ctx from the tracker (which destroys it). */
    void completeCtx(PipelineContext *ctx);

    Deps deps;
    std::deque<PipelineContext*> inFlight;
};

} /* namespace android */

#endif /* HAL_PIPELINE_PIPELINE_THREAD_H */
