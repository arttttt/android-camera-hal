#ifndef HAL_PIPELINE_REQUEST_THREAD_H
#define HAL_PIPELINE_REQUEST_THREAD_H

#include "ThreadBase.h"
#include "EventQueue.h"
#include "PipelineContext.h"

namespace android {

class Pipeline;
class InFlightTracker;

/* Worker thread that drives a Pipeline against PipelineContexts pulled
 * from a bounded queue. Pops a raw PipelineContext pointer (ownership
 * held by InFlightTracker), runs the pipeline, then removes the
 * context from the tracker which triggers its destruction.
 *
 * The queue, pipeline, and tracker are all owned by Camera; this
 * class holds non-owning references. */
class RequestThread : public ThreadBase {
public:
    RequestThread(EventQueue<PipelineContext*> *queue,
                  Pipeline *pipeline,
                  InFlightTracker *tracker);
    ~RequestThread() override;

protected:
    void threadLoop() override;

private:
    EventQueue<PipelineContext*> *queue;
    Pipeline                     *pipeline;
    InFlightTracker              *tracker;
};

} /* namespace android */

#endif /* HAL_PIPELINE_REQUEST_THREAD_H */
