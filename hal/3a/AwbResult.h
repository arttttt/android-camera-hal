#ifndef HAL_3A_AWB_RESULT_H
#define HAL_3A_AWB_RESULT_H

#include <stdint.h>

#include <experimental/optional>

#include "CcmQ10.h"
#include "WbGains.h"

namespace android {

/* Per-frame output of `Awb::process`.
 *
 * - `gains` (when present) — push to `IspPipeline::setWbGains`. The
 *   shader's WB stage applies on the next `vkQueueSubmit` with zero
 *   silicon delay; no DelayedControls timing dance.
 * - `ccm` (when present) — copy into the shared `Camera::mCcmQ10`
 *   buffer the demosaic shader reads by pointer. CCM updates are
 *   gated on the same conditions as gain updates (no CCM drift
 *   while WB is frozen would otherwise produce visible hue shifts
 *   without matching gain change).
 * - `estCct` (Kelvin) — diagnostic / metadata only, not consumed
 *   by any downstream stage. Reported as `ANDROID_SENSOR_REFERENCE_*`-
 *   adjacent metadata in the future (currently unused).
 * - `state` is one of `ANDROID_CONTROL_AWB_STATE_*` for result
 *   metadata.
 *
 * Empty result (no `gains`, no `ccm`, `state == 0`) means AWB
 * skipped this tick — manual mode, lock, or AF sweep. Coordinator
 * routes it by doing nothing. */
struct AwbResult {
    std::experimental::optional<WbGains>  gains;
    std::experimental::optional<CcmQ10>   ccm;
    int                                   estCct = 0;
    uint8_t                               state  = 0;
    /* Number of patches that passed the saturation / noise-floor
     * filter on this tick. -1 = controller didn't run (manual mode,
     * lock, or scene below floor); >= 0 = ran, value matches the
     * gate input. Used by the throttled diagnostic log to surface
     * gray-world confidence per frame. */
    int                                   validPatchCount = -1;
};

} /* namespace android */

#endif /* HAL_3A_AWB_RESULT_H */
