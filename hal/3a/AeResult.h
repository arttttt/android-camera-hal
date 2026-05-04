#ifndef HAL_3A_AE_RESULT_H
#define HAL_3A_AE_RESULT_H

#include <stdint.h>

#include "sensor/DelayedControls.h"

namespace android {

/* Per-frame output of `AutoExposureController::process`.
 *
 * - `batch` carries `EXPOSURE` / `GAIN` slots in `DelayedControls`
 *   semantics: per-id `has[]` flag + value in driver units. The
 *   coordinator pushes this into `DelayedControls` at slot
 *   `request.frameNumber + sensorCfg.controlDelay[id]` so the write
 *   lands on the right future frame.
 * - `state` is one of `ANDROID_CONTROL_AE_STATE_*` for direct
 *   placement into result metadata.
 * - `converged` is the boolean signal AF reads to gate continuous-
 *   mode retriggers (don't retrigger while AE is still chasing
 *   exposure — brightness changes from AE catching up would look
 *   like scene changes). It's a separate field rather than just a
 *   derived `state == CONVERGED` flag because the controller's
 *   internal convergence concept (dead-band hold count) doesn't
 *   exactly match the framework's broader CONVERGED state — keep
 *   the two semantically distinct so each consumer reads what it
 *   actually wants.
 *
 * An empty result (`!batch.has[*]` and `state == 0`) is the
 * "controller did not run this tick" sentinel — happens when AE is
 * locked or AF is sweeping. The coordinator routes a non-result by
 * doing nothing (no DelayedControls push, no state metadata
 * override). */
struct AeResult {
    DelayedControls::Batch  batch;
    uint8_t                 state;
    bool                    converged;
    /* Diagnostic-only — IQM of the top 2% post-WB max-of-channels
     * patches inside the AE focus ROI on this tick. The highlight-
     * protection candidate (`gain_highlight = highlightCap / iqm`)
     * is internal to the controller; this field surfaces the metric
     * to the per-32-frame log for tuning visibility. Zero when the
     * controller did not run this tick. */
    float                   iqmHighlight;

    /* Relative-scale scene brightness in lux units, derived from
     * the AE-converged exposure and the current scene luma against
     * a per-sensor calibration anchor. Zero when the tuning has no
     * `active.ae.luxAnchor` block, or when the controller did not
     * run on this tick. The Bayes AWB consumes this to interpolate
     * its lux-conditioned CT prior; gray-world ignores it. */
    float                   luxIndex;

    AeResult()
        : batch{}, state(0), converged(false),
          iqmHighlight(0.f), luxIndex(0.f) {}
};

} /* namespace android */

#endif /* HAL_3A_AE_RESULT_H */
