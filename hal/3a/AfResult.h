#ifndef HAL_3A_AF_RESULT_H
#define HAL_3A_AF_RESULT_H

#include <stdint.h>

#include "Optional.h"

namespace android {

/* Per-frame output of `AutoFocusController::process`.
 *
 * - `vcmPosition` (when present) — VCM position in driver units the
 *   lens has to move to now; the coordinator writes it on the same
 *   tick. Most frames carry none — the lens moves at sweep starts and
 *   steps, at the commit, and for manual focus.
 * - `sweeping` is true on every tick a sweep is running, passive or
 *   triggered. The coordinator holds AE / AWB locked while it is: a
 *   level, not a pair of start / complete edges, so a sweep cancelled
 *   and restarted within one tick keeps the locks instead of handing
 *   the coordinator "locked" and "unlocked" at once.
 * - `state` is one of `ANDROID_CONTROL_AF_STATE_*` for result
 *   metadata. Reported every tick the controller runs. */
struct AfResult {
    Optional<int32_t>  vcmPosition;
    bool               sweeping = false;
    uint8_t            state    = 0;
};

} /* namespace android */

#endif /* HAL_3A_AF_RESULT_H */
