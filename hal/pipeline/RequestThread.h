#ifndef HAL_PIPELINE_REQUEST_THREAD_H
#define HAL_PIPELINE_REQUEST_THREAD_H

#include "ThreadBase.h"
#include "EventQueue.h"
#include "PipelineContext.h"

namespace android {

class Pipeline;
class InFlightTracker;

/* Worker thread that drives a Pipeline against PipelineContexts pulled
 * from a bounded queue, then hands the context off to a downstream
 * queue (PipelineThread). Ownership stays in the InFlightTracker; this
 * class just marshals raw pointers between queues. pushBlocking
 * paces the upstream at the downstream's maxInFlight capacity.
 *
 * The queues, pipeline, and tracker are all owned by Camera; this
 * class holds non-owning references. */
class RequestThread : public ThreadBase {
public:
    RequestThread(EventQueue<PipelineContext*> *inQueue,
                  Pipeline *pipeline,
                  EventQueue<PipelineContext*> *outQueue,
                  InFlightTracker *tracker);
    ~RequestThread() override;

protected:
    void threadLoop() override;

private:
    EventQueue<PipelineContext*> *inQueue;
    Pipeline                     *pipeline;
    EventQueue<PipelineContext*> *outQueue;
    InFlightTracker              *tracker;
};

} /* namespace android */

#endif /* HAL_PIPELINE_REQUEST_THREAD_H */
