#define LOG_TAG "Cam-StreamConfig"

#include "StreamConfig.h"

#include <stddef.h>
#include <stdint.h>

#include <hardware/gralloc.h>
#include <utils/Log.h>

namespace android {

namespace {

/* Number of gralloc buffers the framework may hold per stream. Four
 * covers current preview / still-capture cadences; raise once the
 * request-queue refactor (Tier 3) introduces real pipelining. */
constexpr uint32_t kMaxBuffersPerStream = 4;

} /* namespace */

status_t StreamConfig::normalize(camera3_stream_configuration_t *streamList,
                                  unsigned *v4l2Width, unsigned *v4l2Height) {
    camera3_stream_t *inStream = NULL;
    unsigned width  = 0;
    unsigned height = 0;

    for (size_t i = 0; i < streamList->num_streams; ++i) {
        camera3_stream_t *newStream = streamList->streams[i];

        if (newStream->stream_type == CAMERA3_STREAM_INPUT ||
            newStream->stream_type == CAMERA3_STREAM_BIDIRECTIONAL) {
            if (inStream) {
                ALOGE("Only one input/bidirectional stream allowed "
                      "(previous is %p, this %p)", inStream, newStream);
                return BAD_VALUE;
            }
            inStream = newStream;
        }

        /* IMPLEMENTATION_DEFINED is a framework-side alias — we pick the
         * concrete format based on stream intent. Video-encoder consumers
         * need planar YUV; everything else gets RGBA. */
        if (newStream->format == HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED) {
            if (newStream->usage & GRALLOC_USAGE_HW_VIDEO_ENCODER)
                newStream->format = HAL_PIXEL_FORMAT_YCbCr_420_888;
            else
                newStream->format = HAL_PIXEL_FORMAT_RGBA_8888;
        }

        if (newStream->usage & GRALLOC_USAGE_HW_CAMERA_ZSL) {
            ALOGE("ZSL not supported. Add camera.disable_zsl_mode=1 to build.prop");
            return BAD_VALUE;
        }

        /* Preserve original usage before rewrite — the framework sets
         * HW_VIDEO_ENCODER on video streams and that drives our
         * resolution pick below. */
        uint32_t origUsage = newStream->usage;
        ALOGD("Stream[%zu]: %ux%u fmt=0x%x usage=0x%x", i,
              newStream->width, newStream->height, newStream->format, origUsage);

        /* Per camera3.h, HAL writes the final usage flags into the
         * stream descriptor. The framework then forwards them to the
         * consumer-side gralloc allocator (see Camera3OutputStream's
         * setConsumerUsageBits) so the buffer is allocated with a
         * memory layout that satisfies BOTH consumer's needs (e.g.
         * HW_TEXTURE for SurfaceTexture preview, HW_VIDEO_ENCODER
         * for the encoder's tiled input) and ours.
         *
         * Overwriting consumer flags with SW_WRITE_OFTEN — what we
         * used to do — silently downgraded gralloc to a linear,
         * CPU-only layout. Apps still ran on HAL3.0 because the
         * Camera1→Camera2 emulation layer fixed the surfaces up
         * itself before our HAL path; on HAL3.2 native-Camera2 we
         * are the final source of truth and the override breaks
         * preview entirely. It also harms the Vulkan fragment-ROP
         * write path which expects the HW-tiled layout the consumer
         * flags imply.
         *
         * Preserve consumer flags. Add HAL-side flags only where the
         * HAL touches the buffer with the CPU — only BLOB outputs
         * today (libjpeg writes the encoded JPEG into the buffer);
         * RGBA / YUV outputs are written via Vulkan ROP and don't
         * need any extra usage bits.  Same logic for INPUT /
         * BIDIRECTIONAL: ZSL reprocess (when it lands) reads via
         * Vulkan, so SW flags only when the HAL actually CPU-reads. */
        const bool needsCpuWrite = (newStream->stream_type == CAMERA3_STREAM_OUTPUT
                                    || newStream->stream_type == CAMERA3_STREAM_BIDIRECTIONAL)
                                && newStream->format == HAL_PIXEL_FORMAT_BLOB;
        if (needsCpuWrite) newStream->usage = origUsage | GRALLOC_USAGE_SW_WRITE_OFTEN;
        else               newStream->usage = origUsage;
        newStream->max_buffers = kMaxBuffersPerStream;

        /* V4L2 resolution selection:
         *   - HW_VIDEO_ENCODER stream wins (sensor switches to matching
         *     FPS mode, e.g. 720p@90fps).
         *   - Otherwise the largest non-BLOB stream. */
        if (newStream->format != HAL_PIXEL_FORMAT_BLOB) {
            bool isVideo = (origUsage & GRALLOC_USAGE_HW_VIDEO_ENCODER) != 0;
            if (isVideo) {
                width  = newStream->width;
                height = newStream->height;
                ALOGD("Video stream detected: %ux%u", width, height);
            } else if (!width || !height ||
                       (newStream->width * newStream->height > width * height)) {
                width  = newStream->width;
                height = newStream->height;
            }
        }
    }

    /* Fallback: only BLOB streams in the config → pick the largest. */
    if (!width || !height) {
        for (size_t i = 0; i < streamList->num_streams; ++i) {
            camera3_stream_t *s = streamList->streams[i];
            if (s->width * s->height > width * height) {
                width  = s->width;
                height = s->height;
            }
        }
    }

    *v4l2Width  = width;
    *v4l2Height = height;
    return NO_ERROR;
}

}; /* namespace android */
