#ifndef HAL_CAMERA_STATIC_METADATA_H
#define HAL_CAMERA_STATIC_METADATA_H

#include <stddef.h>
#include <system/camera_metadata.h>

namespace android {

class V4l2Device;

/* Builds the ANDROID_* static characteristics blob for a single camera.
 * Pure builder — holds no state, called once per camera on first
 * cameraInfo() query. Caller owns the returned metadata pointer and is
 * responsible for releasing it (free_camera_metadata). */
class CameraStaticMetadata {
public:
    /* dev:              V4L2 device the characteristics are computed from
     *                   (resolutions, sensor size, per-mode min frame duration).
     * facing:           CAMERA_FACING_BACK / _FRONT.
     * jpegBufferSize:   [out] page-aligned JPEG buffer size that callers
     *                   can use to size HAL_PIXEL_FORMAT_BLOB allocations.
     *                   Covers the largest resolution + camera3_jpeg_blob
     *                   footer at 2 bytes/pixel. */
    static camera_metadata_t *build(V4l2Device *dev, int facing,
                                     size_t *jpegBufferSize);
};

}; /* namespace android */

#endif /* HAL_CAMERA_STATIC_METADATA_H */
