#ifndef HAL_PIPELINE_BUFFER_PROCESSOR_H
#define HAL_PIPELINE_BUFFER_PROCESSOR_H

#include <stddef.h>
#include <stdint.h>

#include <utils/Errors.h>

#include "Frame.h"
#include "JpegSnapshot.h"
#include "mochacamera/FrameSettings.h"

namespace mocha::camera { class GrallocPort; }

namespace android {

class IspFrameRecorder;
class IspJpegRing;
class IspYuvReadback;
class PostProcessor;

/* Per-output-buffer processing. Records each output's GPU work on the
 * frame's open ISP recording -- a zero-copy blit for RGBA, an NV12
 * encode for YUV, a copy into the JPEG ring for a BLOB -- and finishes
 * the CPU half once the GPU is done: the YUV repack, the JPEG encode.
 *
 * CPU access to the output buffers goes through the gralloc port; the
 * GPU imports them inside the ISP backend. A BufferProcessor instance
 * is reused across frames; per-frame state is passed through
 * FrameContext. */
class BufferProcessor {
public:
    using Frame  = mocha::camera::Frame;
    using Output = mocha::camera::Frame::Output;

    struct Deps {
        IspFrameRecorder           *recorder;
        IspJpegRing                *jpegRing;
        IspYuvReadback             *yuv;
        PostProcessor              *jpeg;
        mocha::camera::GrallocPort *gralloc;
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

    explicit BufferProcessor(const Deps &deps);

    /* Record one output of a frame whose ISP recording is open
     * (DemosaicBlitStage has called isp->beginFrame); the GPU work is
     * kicked off later by isp->endFrame. The output's acquire fence is
     * used up whatever the outcome: waited on, imported, or closed.
     * An RGBA output gets its release fence from endFrame; a BLOB
     * output its JPEG snapshot. */
    status_t processOne(Output &out, const FrameContext &ctx,
                        uint32_t frameNumber);

    /* The CPU half of a committed frame once its GPU work is done: YUV
     * outputs are libyuv-repacked here. BLOB outputs are the JPEG
     * engine's. */
    void finalizeCpuOutputs(Frame &frame);

    /* Give every BLOB output's JPEG snapshot back so the ISP ring
     * rotates without an encode (a frame that failed or was
     * cancelled). */
    void releaseJpegSnapshots(Frame &frame);

    /* The same for one output, whose encode will not run. */
    void releaseJpegSnapshot(Frame &frame, size_t outputIndex);

    /* Lock the BLOB, encode its snapshot into it with the request's
     * JPEG settings, write the camera3_jpeg_blob trailer, unlock, and
     * give the snapshot back. False when the BLOB holds no image; the
     * snapshot is given back either way. */
    bool finalizeBlobOutput(Output &out,
                            const mocha::camera::FrameSettings::Jpeg &jpeg,
                            uint32_t frameNumber);

private:
    /* Block until the consumer releases the buffer for writing; uses
     * the fence up. */
    status_t waitAcquireFence(Output &out, uint32_t frameNumber);

    bool     tryZeroCopy(Output &out, const FrameContext &ctx);
    status_t recordBlobOutput(Output &out, const FrameContext &ctx,
                              uint32_t frameNumber);
    status_t recordYuvOutput(Output &out, const FrameContext &ctx,
                             uint32_t frameNumber);
    void     finalizeYuvOutput(Output &out, uint32_t frameNumber);

    Deps mDeps;
};

}; /* namespace android */

#endif /* HAL_PIPELINE_BUFFER_PROCESSOR_H */
