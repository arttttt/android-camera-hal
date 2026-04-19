#ifndef HAL_PIPELINE_STREAM_CONFIG_H
#define HAL_PIPELINE_STREAM_CONFIG_H

#include <utils/Errors.h>
#include <hardware/camera3.h>

namespace android {

/* Per-request stream-list normalisation + V4L2 capture-resolution
 * selection. Stateless — a pure function over the framework's stream
 * configuration blob. */
class StreamConfig {
public:
    /* Validate and normalise `streamList` in place:
     *   - At most one CAMERA3_STREAM_INPUT / BIDIRECTIONAL stream.
     *   - Reject any stream that sets GRALLOC_USAGE_HW_CAMERA_ZSL.
     *   - Map HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED → RGBA_8888.
     *   - Rewrite `usage` per stream_type (SW_WRITE_OFTEN /
     *     SW_READ_OFTEN).
     *   - Set `max_buffers`.
     *
     * Pick the V4L2 capture resolution:
     *   - An HW_VIDEO_ENCODER stream wins (the sensor locks to the
     *     matching FPS mode, e.g. 720p@90fps).
     *   - Otherwise the largest non-BLOB stream.
     *   - Otherwise the largest BLOB stream.
     *
     * Returns NO_ERROR on success; BAD_VALUE on validation failures
     * (two input streams, ZSL usage). */
    static status_t normalize(camera3_stream_configuration_t *streamList,
                              unsigned *v4l2Width, unsigned *v4l2Height);
};

}; /* namespace android */

#endif /* HAL_PIPELINE_STREAM_CONFIG_H */
