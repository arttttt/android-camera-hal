#ifndef HAL_METADATA_RESULT_METADATA_BUILDER_H
#define HAL_METADATA_RESULT_METADATA_BUILDER_H

#include <stdint.h>
#include <camera/CameraMetadata.h>

#include "3a/AutoFocusController.h"

namespace android {

struct SensorConfig;

/* Populates the result-only keys on a per-frame CameraMetadata blob:
 * timestamp, applied exposure/gain, AF state echo, AE/AWB state
 * derivation, CAPTURE_INTENT echo, static lens info.
 *
 * Reads AE_MODE / AWB_MODE / AWB_LOCK / CAPTURE_INTENT already present
 * in `cm` (the request metadata) and echoes them back. */
class ResultMetadataBuilder {
public:
    struct FrameState {
        int64_t                     timestampNs;
        uint32_t                    frameNumber;
        int32_t                     appliedExposureUs;
        int32_t                     appliedGain;
        AutoFocusController::Report af;
    };

    static void build(CameraMetadata &cm, const FrameState &fs,
                      const SensorConfig &cfg);
};

}; /* namespace android */

#endif /* HAL_METADATA_RESULT_METADATA_BUILDER_H */
