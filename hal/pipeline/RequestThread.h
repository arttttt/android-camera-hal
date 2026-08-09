#ifndef HAL_PIPELINE_REQUEST_THREAD_H
#define HAL_PIPELINE_REQUEST_THREAD_H

#include "ThreadBase.h"
#include "EventQueue.h"
#include "PipelineContext.h"

namespace android {

class Pipeline;

/* Worker thread that drives a Pipeline against PipelineContexts pulled
 * from a bounded queue, then hands the context off to a downstream
 * queue (PipelineThread). Ownership stays in the InFlightTracker; this
 * class just marshals raw pointers between queues. pushBlocking
 * paces the upstream at the downstream's maxInFlight capacity.
 *
 * It takes no tracker of its own, by design: a context stays registered
 * for the whole hop, and on stop it is left there for closeDevice's
 * drainAll to error-complete rather than being retired here.
 *
 * The queues and the pipeline are owned by Camera; this class holds
 * non-owning references. */
class RequestThread : public ThreadBase {
public:
    RequestThread(EventQueue<PipelineContext*> *inQueue,
                  Pipeline *pipeline,
                  EventQueue<PipelineContext*> *outQueue);
    ~RequestThread() override;

protected:
    void threadLoop() override;

private:
    EventQueue<PipelineContext*> *inQueue;
    Pipeline                     *pipeline;
    EventQueue<PipelineContext*> *outQueue;
};

} /* namespace android */

#endif /* HAL_PIPELINE_REQUEST_THREAD_H */
