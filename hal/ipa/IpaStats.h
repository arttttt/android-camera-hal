#ifndef HAL_IPA_STATS_H
#define HAL_IPA_STATS_H

#include <stdint.h>

namespace android {

/* Layout of the statistics buffer NeonStatsEncoder produces on the
 * StatsWorker thread and StatsProcessStage forwards to the IPA.
 *
 * ~4.5 KB total, filled from a single pass (or a `phaseCount`
 * progressive sequence of partial passes) over the raw Bayer slot —
 * pre-WB / pre-CCM space, matching the libcamera IPU3 / rkisp1
 * convention. Consumers must account for the raw domain when
 * interpreting rgbMean. */
struct IpaStats {
    static const int HIST_BINS   = 128;
    static const int PATCH_X     = 16;
    static const int PATCH_Y     = 16;

    /* Focus-metric region of interest, expressed as a half-open
     * patch-grid rectangle [LO, HI) on each axis. Default covers the
     * centre 8×8 patches — 50 % of each frame dimension, the same band
     * the AF controller integrates `focusMetric` over. The encoder
     * accepts this as a runtime argument; tap-to-focus would publish
     * a different rectangle per request once that path is wired up.
     * Acts as a single source of truth: AF uses the same values for
     * its centre-only ROI sums. */
    static const int FOCUS_ROI_PX_LO = 4;
    static const int FOCUS_ROI_PX_HI = 12;
    static const int FOCUS_ROI_PY_LO = 4;
    static const int FOCUS_ROI_PY_HI = 12;

    /* Rec. 601 luma, bin = clamp(luma * HIST_BINS, 0, HIST_BINS-1).
     * uint32_t per bin gives > 4e9 headroom vs 1920*1080 pixels. */
    uint32_t lumaHist[HIST_BINS];

    /* Per-patch mean RGB in [0,1]. Divided by patch pixel count in
     * the shader's tree reduction (not here). */
    float rgbMean[PATCH_Y][PATCH_X][3];

    /* Per-patch focus metric, normalised against per-pixel signal
     * energy: `Σ(Gx² + Gy²) / Σ I²` over the green sub-lattice.
     * Both numerator and denominator scale as brightness², so the
     * ratio is exposure-invariant by construction — a sweep that
     * runs while AE is still adjusting still produces a comparable
     * score curve. Magnitude is dimensionless, typically ranging
     * from ~0.001 (out-of-focus or texture-less) to ~50 (sharp,
     * high-frequency detail). Sole AF input. */
    float focusMetric[PATCH_Y][PATCH_X];
};

} /* namespace android */

#endif /* HAL_IPA_STATS_H */
