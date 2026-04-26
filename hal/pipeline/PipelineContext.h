#ifndef HAL_PIPELINE_PIPELINE_CONTEXT_H
#define HAL_PIPELINE_PIPELINE_CONTEXT_H

#include <atomic>
#include <stdint.h>
#include <vector>

#include <camera/CameraMetadata.h>

#include "CaptureRequest.h"
#include "JpegSnapshot.h"
#include "V4l2Device.h"

namespace android {

/* Per-frame state that travels through pipeline stages.
 *
 * Constructed from a deep-copied CaptureRequest on the binder thread,
 * then moved through pipelines on one or more worker threads. Stages
 * read/write fields to accumulate progress; the owning InFlightTracker
 * destroys the context after the result has been dispatched.
 *
 * Short of the `request` input, every field has a meaningful default
 * for "stage has not yet populated this". Stages that depend on a
 * field should assert that an upstream stage has populated it.
 *
 * All time fields are CLOCK_MONOTONIC nanoseconds (systemTime()). */
struct PipelineContext {
    uint32_t       sequence;
    CaptureRequest request;

    /* Populated by CaptureStage. */
    const V4l2Device::VBuffer *bayerFrame;
    int                        cropX;
    int                        cropY;
    int                        cropW;
    int                        cropH;

    /* Populated by DemosaicBlitStage. Vectors indexed in lockstep with
     * request.outputBuffers — element i corresponds to output i. */
    std::vector<int>  outputReleaseFences;
    std::vector<int>  outputStatuses;
    std::vector<bool> outputNeedsFinalUnlock;

    /* JPEG snapshots for BLOB outputs — only populated for output indices
     * whose stream format is HAL_PIXEL_FORMAT_BLOB. ringSlot < 0 means
     * "not a BLOB output" or "blitToJpegCpu failed". finalizeCpuOutputs
     * encodes + releases. */
    std::vector<JpegSnapshot> outputJpegSnapshots;

    /* Echoed back in result metadata. */
    int32_t appliedExposureUs;
    int32_t appliedGain;

    /* Timestamps for PERF diagnostics. */
    int64_t tAccepted;
    int64_t tShutter;
    int64_t tBayerDq;
    int64_t tResultSent;

    /* Non-zero means this frame is being aborted. ResultDispatchStage
     * (which runs even on error) emits notify(ERROR) + error-status
     * buffers when this is set. */
    int errorCode;

    /* Submit-level sync_fds from vkGetFenceFdKHR, one per GPU submit
     * issued for this context. The thread that drives DemosaicBlit
     * hands these to the PipelineThread poll set; completion signals
     * slot reuse and, once the vector drains, ResultDispatch. */
    std::vector<int> pendingFenceFds;

    /* Per-ctx readiness flags consumed by ResultThread.
     *
     * jpegPending: count of BLOB outputs whose libjpeg encode has not
     *              finished yet. PipelineThread sets the count to the
     *              number of BLOBs in the request just before posting
     *              jobs; JpegWorker decrements on each completion.
     *              ResultThread refuses to dispatch the ctx until it
     *              hits 0.
     *
     * Loaded by ResultThread, stored by PipelineThread + JpegWorker —
     * std::atomic guarantees the reader sees a coherent value without
     * locking. seq_cst is fine; updates are rare. */
    std::atomic<int> jpegPending;

    PipelineContext()
        : sequence(0),
          bayerFrame(nullptr),
          cropX(0), cropY(0), cropW(0), cropH(0),
          appliedExposureUs(0), appliedGain(0),
          tAccepted(0), tShutter(0), tBayerDq(0), tResultSent(0),
          errorCode(0),
          jpegPending(0) {}

    PipelineContext(PipelineContext&&) = default;
    PipelineContext& operator=(PipelineContext&&) = default;
    PipelineContext(const PipelineContext&) = delete;
    PipelineContext& operator=(const PipelineContext&) = delete;
};

} /* namespace android */

#endif /* HAL_PIPELINE_PIPELINE_CONTEXT_H */
