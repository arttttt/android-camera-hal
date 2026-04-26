#ifndef HAL_PIPELINE_BUFFER_PROCESSOR_H
#define HAL_PIPELINE_BUFFER_PROCESSOR_H

#include <stddef.h>
#include <stdint.h>

#include <utils/Errors.h>
#include <hardware/camera3.h>
#include <camera/CameraMetadata.h>

#include "CaptureRequest.h"
#include "JpegSnapshot.h"

namespace android {

class IspPipeline;
class PostProcessor;
struct PipelineContext;

/* Per-output-buffer processing. Takes a just-dequeued V4L2 Bayer
 * frame and emits either a zero-copy RGBA gralloc (preview) or a
 * libjpeg-encoded BLOB (still capture).
 *
 * A BufferProcessor instance is reused across frames; per-frame state
 * is passed through FrameContext. */
class BufferProcessor {
public:
    struct Deps {
        IspPipeline   *isp;
        PostProcessor *jpeg;
        const size_t  *jpegBufferSize;  /* single value per session, set at configureStreams */
    };

    struct FrameContext {
        const uint8_t *frameBuf;      /* NULL on the DMABUF capture path */
        int            frameSlotIdx;  /* V4L2 ring slot, valid iff frameBuf == NULL */
        uint32_t       pixFmt;
        unsigned       resW;
        unsigned       resH;
        int            cropX;
        int            cropY;
        int            cropW;
        int            cropH;
    };

    struct OutputState {
        bool needsFinalUnlock;
    };

    explicit BufferProcessor(const Deps &deps);

    /* Process one output buffer of a frame whose ISP recording is open
     * (DemosaicBlitStage has called isp->beginFrame). On NO_ERROR the
     * output's blit / encode has been recorded; the actual GPU work is
     * kicked off later by isp->endFrame.
     *
     * releaseFenceOut: address of the slot in ctx.outputReleaseFences for
     *                  this output. RGBA outputs stash the address inside
     *                  the ISP and endFrame writes the per-output sync_fd
     *                  there. YUV / BLOB leave it -1 (CPU finalize is
     *                  synchronous in the consumer thread).
     * jpegSnapshot:    [out, may be NULL] address where this output's
     *                  JpegSnapshot lands, populated for BLOB outputs
     *                  via blitToJpegCpu. Other formats leave it
     *                  default-initialised (ringSlot == -1). */
    status_t processOne(const camera3_stream_buffer &srcBuf,
                        const FrameContext &ctx,
                        const CameraMetadata &cm,
                        uint32_t frameNumber,
                        OutputState *state,
                        int *releaseFenceOut,
                        JpegSnapshot *jpegSnapshot);

    /* Post-fence-reap CPU finalize for outputs whose GPU half ran into a
     * host-mapped backend buffer (YUV NV12). Called by PipelineThread on
     * a successfully completed frame, before stats and result dispatch.
     * No-op when no YUV output is in the request. */
    void finalizeCpuOutputs(PipelineContext &ctx);

private:
    /* Block until the consumer releases srcBuf for writing. */
    status_t waitAcquireFence(const camera3_stream_buffer &srcBuf,
                              uint32_t frameNumber);

    /* Record an RGBA blit on the open ISP recording. acquireFence
     * (sync_fd from srcBuf) is imported as a binary VkSemaphore so the
     * eventual submit GPU-waits on framework readiness. The releaseFenceOut
     * pointer is stashed inside the ISP and populated by endFrame. */
    bool     tryZeroCopy(const camera3_stream_buffer &srcBuf,
                         const FrameContext &ctx,
                         OutputState *state,
                         int *releaseFenceOut);

    /* Lock the gralloc buffer for CPU write; writes the mapped pointer
     * to *outBuf. Used by the BLOB path — libjpeg writes into it. */
    status_t lockSwWrite(const camera3_stream_buffer &srcBuf,
                         uint32_t frameNumber,
                         uint8_t **outBuf);

    /* Record a vkCmdCopyImageToBuffer from the ISP scratch into a JPEG
     * ring slot. The actual encode runs in finalizeBlobOutput after the
     * fence reaps. */
    status_t recordBlobOutput(const camera3_stream_buffer &srcBuf,
                               uint32_t frameNumber,
                               JpegSnapshot *snapshotOut);

    /* Lock the BLOB gralloc, libjpeg-encode from the snapshot, write the
     * camera3_jpeg_blob trailer, unlock, release the snapshot. Called
     * from finalizeCpuOutputs once the frame's submit fence has signalled. */
    void     finalizeBlobOutput(const CaptureRequest::Buffer &outBuf,
                                 const CameraMetadata &metadata,
                                 const JpegSnapshot &snap,
                                 uint32_t frameNumber);

    /* Record a GPU NV12 encode dispatch on the open ISP recording. CPU
     * wait on srcBuf.acquire_fence happens here so the subsequent
     * finalizeYuvOutput can lock the gralloc without blocking. */
    status_t recordYuvOutput(const camera3_stream_buffer &srcBuf,
                              const FrameContext &ctx,
                              uint32_t frameNumber);

    /* Lock the gralloc, libyuv-repack from the ISP host buffer, unlock.
     * Called from finalizeCpuOutputs once the frame's submit fence has
     * signalled. */
    void     finalizeYuvOutput(const CaptureRequest::Buffer &outBuf,
                                uint32_t frameNumber);

    Deps mDeps;
};

}; /* namespace android */

#endif /* HAL_PIPELINE_BUFFER_PROCESSOR_H */
