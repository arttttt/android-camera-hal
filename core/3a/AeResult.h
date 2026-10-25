#ifndef HAL_3A_AE_RESULT_H
#define HAL_3A_AE_RESULT_H

#include <stdint.h>

#include "sensor/DelayedControls.h"

namespace android {

/* Per-frame output of `AutoExposureController::process`.
 *
 * - `batch` carries `EXPOSURE` / `GAIN` slots in `DelayedControls`
 *   semantics: per-id `has[]` flag + value in driver units. The
 *   coordinator publishes it as the exposure target, which the next
 *   request in auto AE takes.
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
 * An empty result (`!batch.has[*]`) is the
 * "controller did not run this tick" sentinel — happens when AE is
 * locked or AF is sweeping. The coordinator routes a non-result by
 * doing nothing (no exposure target update, no state metadata
 * override). */
struct AeResult {
    DelayedControls::Batch  batch;
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

    /* The metered luma this scene would reach at the envelope's
     * longest exposure and highest gain, as a fraction of the AE
     * setpoint (below 1: AE cannot bring it up to the target). Scene
     * light normalised by exposure, so it holds wherever AE is on its
     * way. AF reads it to skip a sweep in a scene too dark to focus
     * on, and to stop one the scene goes dark under. Measured locked
     * or not; zero with no record of the applied exposure. */
    float                   lightAtMax;

    AeResult()
        : batch{}, converged(false),
          iqmHighlight(0.f), luxIndex(0.f), lightAtMax(0.f) {}
};

} /* namespace android */

#endif /* HAL_3A_AE_RESULT_H */
