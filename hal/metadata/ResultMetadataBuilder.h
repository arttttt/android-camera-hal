#ifndef HAL_METADATA_RESULT_METADATA_BUILDER_H
#define HAL_METADATA_RESULT_METADATA_BUILDER_H

#include <stdint.h>
#include "CameraMetadataCompat.h"

#include "mochacamera/Results.h"

namespace android {

/* Writes result-only keys onto a frame's metadata, which starts as the
 * request's own settings (the echo). The core's 3A report is turned
 * into tags here; the states in it are the core's, not re-derived.
 * Keys split into subsets so each partial result carries its own:
 *
 *   AWB partial (counter 1, from the 3A tick, no buffers)
 *     ANDROID_CONTROL_AWB_STATE, ANDROID_CONTROL_AWB_MODE
 *
 *   AE partial (counter 2, from the 3A tick, no buffers)
 *     ANDROID_CONTROL_AE_STATE, ANDROID_CONTROL_AE_MODE,
 *     ANDROID_SENSOR_EXPOSURE_TIME, ANDROID_SENSOR_SENSITIVITY,
 *     ANDROID_SENSOR_FRAME_DURATION
 *
 *   Final (counter 3, with the buffers): all of the above, and
 *     ANDROID_CONTROL_AF_STATE, ANDROID_CONTROL_AF_MODE,
 *     ANDROID_LENS_FOCUS_DISTANCE, ANDROID_SENSOR_TIMESTAMP,
 *     ANDROID_SYNC_FRAME_NUMBER, ANDROID_CONTROL_CAPTURE_INTENT,
 *     ANDROID_LENS_APERTURE, ANDROID_LENS_FOCAL_LENGTH,
 *     ANDROID_FLASH_STATE
 *
 *   Every one of them, as each starts from the request's settings:
 *     ANDROID_SCALER_CROP_REGION, ANDROID_CONTROL_AF_REGIONS -- what
 *     the frame used, in place of the request's, so the partials and
 *     the final agree */
class ResultMetadataBuilder {
public:
    static void buildAwb(CameraMetadata &cm, const mocha::camera::Core3AResult &r);
    static void buildAe (CameraMetadata &cm, const mocha::camera::Core3AResult &r);
    static void buildAf (CameraMetadata &cm, const mocha::camera::Core3AResult &r);
    static void buildRegions(CameraMetadata &cm, const mocha::camera::Core3AResult &r);
    static void buildBase(CameraMetadata &cm, int64_t timestampNs,
                          uint32_t frameNumber, float focalLengthMm);

    /* The final result's set. */
    static void buildFinal(CameraMetadata &cm, const mocha::camera::Core3AResult &r,
                           int64_t timestampNs, uint32_t frameNumber,
                           float focalLengthMm);
};

}; /* namespace android */

#endif /* HAL_METADATA_RESULT_METADATA_BUILDER_H */
