#ifndef HAL_CAMERA_STATIC_METADATA_H
#define HAL_CAMERA_STATIC_METADATA_H

#include <stddef.h>
#include <stdint.h>
#include <system/camera_metadata.h>

#include "mochacamera/Capability.h"

namespace android {


/* `ANDROID_REQUEST_PARTIAL_RESULT_COUNT` — maximum number of
 * `process_capture_result` calls per frame. Result metadata is split
 * across this many partials, each carrying its own subset of the
 * keys advertised in `ANDROID_REQUEST_AVAILABLE_RESULT_KEYS`; the
 * final partial (counter == kPartialResultCount) carries the buffer
 * pointers and any base capture metadata. Currently 3 — one for AWB
 * results, one for AE, one for AF + base metadata + buffers.
 *
 * The result sink numbers the partial results by it; it is advertised
 * verbatim in static characteristics. */
constexpr int32_t kPartialResultCount = 3;

/* Builds the ANDROID_* static characteristics blob for a single camera
 * from the core's Capability, adding what is Camera3 policy rather than
 * capability (the per-stage OFF modes, the hardware level, the key
 * lists). Caller owns the returned metadata (free_camera_metadata). */
class CameraStaticMetadata {
public:
    static camera_metadata_t *build(const mocha::camera::Capability &cap);
};

}; /* namespace android */

#endif /* HAL_CAMERA_STATIC_METADATA_H */
