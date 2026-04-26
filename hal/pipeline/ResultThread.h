#ifndef HAL_PIPELINE_RESULT_THREAD_H
#define HAL_PIPELINE_RESULT_THREAD_H

#include <cstddef>

#include "ThreadBase.h"
#include "EventQueue.h"
#include "PipelineContext.h"

namespace android {

class PipelineStage;
class BayerSource;
class InFlightTracker;

/* Dispatch side of the async pipeline.
 *
 * Consumes PipelineContexts that PipelineThread offloads after fence
 * reap + stats + CPU finalize. Runs ResultDispatchStage exactly once
 * per ctx, flushes pending Bayer releases, and removes the ctx from
 * the tracker (which destroys it).
 *
 * Single-threaded by construction — Camera3 result ordering is
 * preserved by the queue's FIFO and this thread's serial execution.
 *
 * Stop semantics: when stop() is signalled, threadLoop drains any
 * remaining queued ctxs (marking them ERROR_REQUEST if they hadn't
 * been flagged already) so framework-owned buffers return cleanly. */
class ResultThread : public ThreadBase {
public:
    struct Deps {
        EventQueue<PipelineContext*> *queue;
        PipelineStage                *resultDispatch;
        BayerSource                  *bayerSource;
        InFlightTracker              *tracker;
    };

    explicit ResultThread(const Deps &deps);
    ~ResultThread() override;

protected:
    void threadLoop() override;

private:
    void completeCtx(PipelineContext *ctx);

    Deps deps;
};

} /* namespace android */

#endif /* HAL_PIPELINE_RESULT_THREAD_H */
