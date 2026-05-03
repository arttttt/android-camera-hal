#ifndef HAL_3A_AF_RESULT_H
#define HAL_3A_AF_RESULT_H

#include <stdint.h>

#include <experimental/optional>

namespace android {

/* Per-frame output of `AutoFocusController::process`.
 *
 * - `vcmPosition` (when present) — VCM position in driver units,
 *   written via `V4l2Device::setControl(V4L2_CID_FOCUS_ABSOLUTE,
 *   ...)` against the focuser subdev. Most frames carry no write —
 *   VCM is moved only at scan steps.
 * - `startSweep` flips true on the frame the controller transitions
 *   into an active scan; the coordinator uses this to suppress
 *   subsequent AE / AWB controller invocations until the matching
 *   `sweepComplete` arrives. Implicit lock — replaces the explicit
 *   `setAeLock(true)` / `setAwbLocked(true)` cross-controller calls
 *   the previous design used.
 * - `sweepComplete` flips true on the frame the scan reaches its
 *   terminal state (Settle entry); coordinator resumes AE / AWB
 *   from the next tick.
 * - `state` is one of `ANDROID_CONTROL_AF_STATE_*` for result
 *   metadata. Reported every tick the controller runs.
 *
 * `startSweep` and `sweepComplete` are edge signals, not levels —
 * the coordinator latches phase based on them rather than re-
 * reading state every frame. Avoids the "state == ACTIVE_SCAN"
 * polling pattern that conflates "AF moving the lens this frame"
 * (false on most frames during a scan) with "AF in scan mode" (true
 * for the whole scan duration). */
struct AfResult {
    std::experimental::optional<int32_t>  vcmPosition;
    bool                                  startSweep    = false;
    bool                                  sweepComplete = false;
    uint8_t                               state         = 0;
};

} /* namespace android */

#endif /* HAL_3A_AF_RESULT_H */
