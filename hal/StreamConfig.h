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
     * The capture size is the core's choice (Session::configure), from
     * the streams as this leaves them.
     *
     * Returns NO_ERROR on success; BAD_VALUE on validation failures
     * (two input streams, ZSL usage). */
    static status_t normalize(camera3_stream_configuration_t *streamList);
};

}; /* namespace android */

#endif /* HAL_PIPELINE_STREAM_CONFIG_H */
