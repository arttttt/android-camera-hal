#ifndef HAL_CAMERA_STATIC_METADATA_H
#define HAL_CAMERA_STATIC_METADATA_H

#include <stddef.h>
#include <stdint.h>
#include <system/camera_metadata.h>

namespace android {

class V4l2Device;
class SensorTuning;
struct SensorConfig;

/* `ANDROID_REQUEST_PARTIAL_RESULT_COUNT` — maximum number of
 * `process_capture_result` calls per frame. Result metadata is split
 * across this many partials, each carrying its own subset of the
 * keys advertised in `ANDROID_REQUEST_AVAILABLE_RESULT_KEYS`; the
 * final partial (counter == kPartialResultCount) carries the buffer
 * pointers and any base capture metadata. Currently 3 — one for AWB
 * results, one for AE, one for AF + base metadata + buffers.
 *
 * Producers (the IPA tick, the result-dispatch stage) emit through
 * `PartialEmitter`; this constant is the upper-bound counter and is
 * advertised verbatim in static characteristics. */
constexpr int32_t kPartialResultCount = 3;

/* Builds the ANDROID_* static characteristics blob for a single camera.
 * Pure builder — holds no state, called once per camera on first
 * cameraInfo() query. Caller owns the returned metadata pointer and is
 * responsible for releasing it (free_camera_metadata). */
class CameraStaticMetadata {
public:
    /* dev:              V4L2 device the characteristics are computed from
     *                   (resolutions, per-mode min frame duration,
     *                   gain / exposure ranges via QUERYCTRL).
     * facing:           CAMERA_FACING_BACK / _FRONT.
     * tuning:           per-module hardware info (physical size, focal
     *                   length, min focus distance). nullptr or
     *                   !isLoaded() falls back to compile-time defaults.
     * sensorCfg:        sensor convention struct — gainUnit /
     *                   kIsoAtUnityGain feed the gain↔ISO conversion in
     *                   SENSITIVITY_RANGE.
     * jpegBufferSize:   [out] page-aligned JPEG buffer size that callers
     *                   can use to size HAL_PIXEL_FORMAT_BLOB allocations.
     *                   Covers the largest resolution + camera3_jpeg_blob
     *                   footer at 2 bytes/pixel. */
    static camera_metadata_t *build(V4l2Device *dev, int facing,
                                     const SensorTuning *tuning,
                                     const SensorConfig &sensorCfg,
                                     size_t *jpegBufferSize);
};

}; /* namespace android */

#endif /* HAL_CAMERA_STATIC_METADATA_H */
