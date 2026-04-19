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
     *   whenever the CPU fallback / AF-readback path produces them.
     *   Reused across iterations to skip redundant demosaic, and read
     *   by the AF sharpness measurement after the per-buffer loop
     *   completes. */
    status_t processOne(const camera3_stream_buffer &srcBuf,
                        const FrameContext &ctx,
                        const CameraMetadata &cm,
                        uint32_t frameNumber,
                        OutputState *state,
                        uint8_t **sharedRgba);

private:
    Deps mDeps;
};

}; /* namespace android */

#endif /* HAL_PIPELINE_BUFFER_PROCESSOR_H */
