#ifndef HAL_PIPELINE_BUFFER_PROCESSOR_H
#define HAL_PIPELINE_BUFFER_PROCESSOR_H

#include <stddef.h>
#include <stdint.h>

#include <utils/Errors.h>
#include <hardware/camera3.h>
#include <camera/CameraMetadata.h>

namespace android {

class IspPipeline;
class JpegEncoder;
class AutoFocusController;

/* Per-output-buffer processing. Takes a just-dequeued V4L2 Bayer
 * frame and emits either a zero-copy RGBA gralloc (preview) or a
 * libjpeg-encoded BLOB (still capture).
 *
 * A BufferProcessor instance is reused across frames; per-frame state
 * is passed through FrameContext. */
class BufferProcessor {
public:
    struct Deps {
        IspPipeline         *isp;
        JpegEncoder         *jpeg;
        AutoFocusController *af;
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
        bool           needZoom;
        size_t         jpegBufferSize;
    };

    struct OutputState {
        bool needsFinalUnlock;
        int  releaseFd;
    };

    explicit BufferProcessor(const Deps &deps);

    /* Process one output buffer. On NO_ERROR, `*state` is populated.
     *
     * sharedRgba: in-out — points at CPU-readable RGBA when an AF
     *   sweep forces a SW_READ lock on the first zero-copy output.
     *   Read by the AF sharpness metric after the per-buffer loop. */
    status_t processOne(const camera3_stream_buffer &srcBuf,
                        const FrameContext &ctx,
                        const CameraMetadata &cm,
                        uint32_t frameNumber,
                        OutputState *state,
                        uint8_t **sharedRgba);

private:
    /* Block until the consumer releases srcBuf for writing. */
    status_t waitAcquireFence(const camera3_stream_buffer &srcBuf,
                              uint32_t frameNumber);

    /* GPU demosaic + optional crop/scale direct into gralloc. On
     * success populates state->releaseFd and, if AF is sweeping,
     * locks srcBuf SW_READ_OFTEN and points *sharedRgba at it.
     * Returns false if processToGralloc failed — that is an error
     * on Bayer hardware and the caller propagates it. */
    bool     tryZeroCopy(const camera3_stream_buffer &srcBuf,
                         const FrameContext &ctx,
                         OutputState *state,
                         uint8_t **sharedRgba);

    /* Lock the gralloc buffer for CPU write; writes the mapped pointer
     * to *outBuf. Used by the BLOB path — libjpeg writes into it. */
    status_t lockSwWrite(const camera3_stream_buffer &srcBuf,
                         uint32_t frameNumber,
                         uint8_t **outBuf);

    /* JPEG BLOB output — delegates to JpegEncoder. */
    void     processBlobOutput(uint8_t *buf,
                               const FrameContext &ctx,
                               const CameraMetadata &cm);

    /* YUV_420_888 output — GPU NV12 via IspPipeline::processToYuv420 +
     * layout repack through libyuv. Locks + unlocks the gralloc buffer
     * internally (like the BLOB path); state->needsFinalUnlock stays
     * true so the outer loop's unlock is a no-op on already-unlocked
     * buffers. */
    status_t processYuvOutput(const camera3_stream_buffer &srcBuf,
                              const FrameContext &ctx,
                              uint32_t frameNumber);

    Deps mDeps;
};

}; /* namespace android */

#endif /* HAL_PIPELINE_BUFFER_PROCESSOR_H */
