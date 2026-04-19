#ifndef HAL_PIPELINE_BUFFER_PROCESSOR_H
#define HAL_PIPELINE_BUFFER_PROCESSOR_H

#include <stddef.h>
#include <stdint.h>

#include <utils/Errors.h>
#include <hardware/camera3.h>
#include <camera/CameraMetadata.h>

namespace android {

class IspPipeline;
class ImageConverter;
class JpegEncoder;
class AutoFocusController;

/* Per-output-buffer processing. Produces a ready-to-return gralloc
 * buffer from a just-dequeued V4L2 frame: zero-copy into gralloc where
 * possible (Bayer RGBA preview), otherwise the CPU RGBA / BLOB JPEG
 * fallback paths.
 *
 * A BufferProcessor instance is reused across frames; per-frame state
 * is passed through FrameContext. */
class BufferProcessor {
public:
    struct Deps {
        IspPipeline         *isp;
        ImageConverter      *converter;
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
        uint8_t       *rgbaScratch;
    };

    struct OutputState {
        bool needsFinalUnlock;
        int  releaseFd;
    };

    explicit BufferProcessor(const Deps &deps);

    /* Process one output buffer. On NO_ERROR, `*state` is populated.
     *
     * sharedRgba: in-out — points at CPU-readable demosaiced pixels
     *   whenever either (a) the CPU fallback path runs (demosaic into
     *   scratch) or (b) an AF sweep forces a SW_READ lock on the first
     *   eligible zero-copy output. Read by the AF sharpness metric
     *   after the per-buffer loop. Zero-copy outputs no longer cache
     *   it for reuse by later iterations — every RGBA stream runs its
     *   own GPU blit independently. */
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

    /* Attempt the GPU→gralloc zero-copy path. Returns true if the
     * buffer is fully serviced (caller skips the CPU path); false if
     * ineligible or if processToGralloc failed. On success, populates
     * state->releaseFd; may also lock the buffer for AF readback. */
    bool     tryZeroCopy(const camera3_stream_buffer &srcBuf,
                         const FrameContext &ctx,
                         OutputState *state,
                         uint8_t **sharedRgba);

    /* Lock the gralloc buffer for CPU write; writes the mapped pointer
     * to *outBuf. Used by the RGBA/BLOB CPU fallback paths. */
    status_t lockSwWrite(const camera3_stream_buffer &srcBuf,
                         uint32_t frameNumber,
                         uint8_t **outBuf);

    /* CPU RGBA output: demosaic (once, cached in *sharedRgba) then
     * zoom-crop or size-adjust into the gralloc buffer. */
    void     processRgbaOutput(const camera3_stream_buffer &srcBuf,
                               uint8_t *buf,
                               const FrameContext &ctx,
                               uint8_t **sharedRgba);

    /* JPEG BLOB output — delegates to JpegEncoder. */
    void     processBlobOutput(uint8_t *buf,
                               const FrameContext &ctx,
                               const CameraMetadata &cm);

    Deps mDeps;
};

}; /* namespace android */

#endif /* HAL_PIPELINE_BUFFER_PROCESSOR_H */
