#ifndef HAL_METADATA_RESULT_METADATA_BUILDER_H
#define HAL_METADATA_RESULT_METADATA_BUILDER_H

#include <stdint.h>
#include <camera/CameraMetadata.h>

#include "3a/AutoFocusController.h"

namespace android {

struct SensorConfig;

/* Populates result-only keys on a per-frame CameraMetadata blob.
 * Keys split into four subsets so the per-controller partial-result
 * emit path can fire each subset on its own partial:
 *
 *   AWB partial (counter 1, IPA tick, no buffers)
 *     ANDROID_CONTROL_AWB_STATE, ANDROID_CONTROL_AWB_MODE
 *
 *   AE partial (counter 2, IPA tick, no buffers)
 *     ANDROID_CONTROL_AE_STATE, ANDROID_CONTROL_AE_MODE,
 *     ANDROID_SENSOR_EXPOSURE_TIME, ANDROID_SENSOR_SENSITIVITY,
 *     ANDROID_SENSOR_FRAME_DURATION
 *
 *   AF partial (counter 3 = final, ResultDispatchStage, with buffers)
 *     ANDROID_CONTROL_AF_STATE, ANDROID_CONTROL_AF_MODE,
 *     ANDROID_LENS_FOCUS_DISTANCE
 *
 *   Base metadata (rolled into counter 3 with AF)
 *     ANDROID_SENSOR_TIMESTAMP, ANDROID_SYNC_FRAME_NUMBER,
 *     ANDROID_CONTROL_CAPTURE_INTENT, ANDROID_LENS_APERTURE,
 *     ANDROID_LENS_FOCAL_LENGTH
 *
 * Each subset reads / echoes the corresponding *_MODE / *_LOCK keys
 * already present in the source metadata (the request settings).
 *
 * `build()` runs all four into one CameraMetadata — kept as an
 * aggregate entry point for the result-dispatch path which needs
 * the full set on the final partial. */
class ResultMetadataBuilder {
public:
    struct FrameState {
        int64_t                     timestampNs;
        uint32_t                    frameNumber;
        int32_t                     appliedExposureUs;
        int32_t                     appliedGain;
        AutoFocusController::Report af;
        bool                        aeConverged;
    };

    static void buildAwbMetadata(CameraMetadata &cm, const FrameState &fs);
    static void buildAeMetadata (CameraMetadata &cm, const FrameState &fs,
                                 const SensorConfig &cfg);
    static void buildAfMetadata (CameraMetadata &cm, const FrameState &fs);
    static void buildBaseMetadata(CameraMetadata &cm, const FrameState &fs);

    /* Aggregate — runs all four subsets in order. Used by
     * ResultDispatchStage on the final partial so the merged result
     * is identical to the pre-split single-emit blob. */
    static void build(CameraMetadata &cm, const FrameState &fs,
                      const SensorConfig &cfg);
};

}; /* namespace android */

#endif /* HAL_METADATA_RESULT_METADATA_BUILDER_H */
