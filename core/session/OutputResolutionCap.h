#ifndef CAMERA_HAL_OUTPUT_RESOLUTION_CAP_H
#define CAMERA_HAL_OUTPUT_RESOLUTION_CAP_H

#include <stdint.h>

namespace android {

/* Advertised output resolution ceiling. The SW ISP (Vulkan demosaic +
 * blit + libjpeg encode) is FPS-bound above 1080p on Tegra K1, and the
 * post-demosaic scratch ring sized for the V4L2 capture mode would
 * grow ~32 MB per slot at 8 MP — too much for the shared nvmap pool
 * on Mocha. Apps see only 16:9 ≤ 1920×1080 in
 * SCALER_AVAILABLE_STREAM_CONFIGURATIONS / JPEG / PROCESSED size lists,
 * so user-facing photo / video resolution pickers stay honest about
 * what the pipeline actually delivers fast. 4:3 modes are intentionally
 * absent; once kernel sensor profiles grow proper 4:3 cropping that
 * side will pick those up.
 *
 * Filter applies to advertised characteristics only — StreamConfig
 * itself stays permissive on what configureStreams accepts so the
 * framework's small system-side surfaces (MediaRecorder thumbnail
 * callback, face-detect inputs) that were never in the advertised set
 * can still pass through to the V4L2 / ISP downscale path. */
namespace OutputResolutionCap {

constexpr int32_t kMaxWidth  = 1920;
constexpr int32_t kMaxHeight = 1080;

inline bool accepts(int32_t width, int32_t height) {
    if (width <= 0 || height <= 0)                return false;
    if (width > kMaxWidth || height > kMaxHeight) return false;
    return (int64_t)width * 9 == (int64_t)height * 16;
}

} /* namespace OutputResolutionCap */

} /* namespace android */

#endif
