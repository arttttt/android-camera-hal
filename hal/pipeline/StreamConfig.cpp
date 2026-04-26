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
    /* HAL3.3 introduced the operation_mode field. We only do the
     * standard pipeline; CONSTRAINED_HIGH_SPEED requires a high-fps
     * sensor mode and a relaxed-metadata path we haven't built. */
    if (streamList->operation_mode != CAMERA3_STREAM_CONFIGURATION_NORMAL_MODE) {
        ALOGE("Unsupported stream-config operation_mode: %u",
              streamList->operation_mode);
        return BAD_VALUE;
    }

    camera3_stream_t *inStream = NULL;
    unsigned width  = 0;
    unsigned height = 0;

    for (size_t i = 0; i < streamList->num_streams; ++i) {
        camera3_stream_t *newStream = streamList->streams[i];

        /* HAL3.3 contract: HAL must inspect rotation and reject what
         * it can't perform. Our blit / encode path doesn't rotate, so
         * everything except 0° is rejected — apps that need rotation
         * apply it themselves at the surface level (or via the JPEG
         * EXIF Orientation tag for stills, which is what Camera3
         * already does for stills today). */
        if (newStream->rotation != CAMERA3_STREAM_ROTATION_0) {
            ALOGE("Stream[%zu]: rotation %d not supported (only ROTATION_0)",
                  i, newStream->rotation);
            return BAD_VALUE;
        }

        /* HAL3.3 data_space: framework sets a colour-space hint per
         * stream (SRGB, JFIF, BT_601_625, …). We produce sRGB-derived
         * RGBA preview, JFIF JPEG and BT.601-limited NV12; depth
         * streams have no sensor backing them and would be silently
         * black, which is worse than a clean rejection. */
        if (newStream->data_space == HAL_DATASPACE_DEPTH) {
            ALOGE("Stream[%zu]: depth dataspace not supported", i);
            return BAD_VALUE;
        }

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
         * RGBA preview keeps consumer-only flags so SurfaceTexture
         * gets a HW-sampleable tiled layout — overriding that broke
         * preview entirely on native Camera2.
         *
         * For BLOB the HAL CPU-writes the encoded JPEG, so add
         * SW_WRITE_OFTEN.
         *
         * For YCbCr_420_888 the consumer (MediaCodec / NVENC) sets
         * HW_VIDEO_ENCODER, which on NVIDIA gralloc maps to a
         * height-padded tiled NV12 (~12 MB per 1080p buffer instead
         * of ~3 MB). Since the HAL writes YUV via the Vulkan ROP
         * path — not the NVENC HW read path — the tiled layout is
         * pure waste on us, and across `kMaxBuffersPerStream`
         * buffers the surplus is enough to push framework gralloc
         * allocations into nvmap-pool OOM during video record on
         * Mocha. Adding SW_WRITE_OFTEN forces gralloc to pick a
         * CPU-writable (= linear-or-near-linear) layout that still
         * satisfies HW_VIDEO_ENCODER consumption; encoder reads from
         * linear NV12 just fine, only marginal HW efficiency cost.
         *
         * Same SW-flag rationale for INPUT / BIDIRECTIONAL once ZSL
         * reprocess lands: Vulkan reads the input, so SW flags only
         * where the HAL itself CPU-touches the buffer. */
        const bool needsCpuWrite = (newStream->stream_type == CAMERA3_STREAM_OUTPUT
                                    || newStream->stream_type == CAMERA3_STREAM_BIDIRECTIONAL)
                                && (newStream->format == HAL_PIXEL_FORMAT_BLOB
                                    || newStream->format == HAL_PIXEL_FORMAT_YCbCr_420_888);
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
