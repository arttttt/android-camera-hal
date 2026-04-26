#ifndef HAL_PIPELINE_JPEG_WORKER_H
#define HAL_PIPELINE_JPEG_WORKER_H

#include <cstddef>

#include "ThreadBase.h"
#include "EventQueue.h"
#include "EventFd.h"

namespace android {

class BufferProcessor;
struct PipelineContext;

/* Async libjpeg encoder. Pulls per-output JPEG jobs from a queue
 * (PipelineThread posts one per BLOB output of a frame), runs the
 * encode out-of-band of the GPU pipeline, and signals ResultThread
 * via a shared completion eventfd when the ctx's outstanding JPEG
 * count reaches zero.
 *
 * One worker per Camera; the queue is the single FIFO across BLOB
 * outputs. With current PIPELINE_MAX_IN_FLIGHT = 1 only one ctx is
 * being recorded by PipelineThread at a time, so encoder backlog is
 * bounded by the framework's BLOB submission rate.
 *
 * Encode itself is delegated to BufferProcessor::finalizeBlobOutput
 * which owns the gralloc lock + libjpeg call + JpegSnapshot release. */
class JpegWorker : public ThreadBase {
public:
    struct Job {
        PipelineContext *ctx;
        std::size_t      outputIndex;
    };

    struct Deps {
        EventQueue<Job> *queue;
        BufferProcessor *bufferProcessor;
        EventFd         *completionWake;
    };

    explicit JpegWorker(const Deps &deps);
    ~JpegWorker() override;

protected:
    void threadLoop() override;

private:
    void runJob(const Job &job);

    Deps deps;
};

} /* namespace android */

#endif /* HAL_PIPELINE_JPEG_WORKER_H */
