#include "GrayWorldAwbController.h"

#include <math.h>
#include <stdint.h>

#include "ipa/IpaStats.h"
#include "sensor/SensorTuning.h"

namespace android {

namespace {

/* Saturation ceiling for per-patch filtering. Patches with any
 * channel above this value are dropped — the missing highlight
 * biases the mean toward the unclipped channel. Algorithm choice,
 * not read from tuning: NVIDIA's CStatsSaturationThreshold
 * represents a different stats-stage metric, and 0.95 in our
 * [0, 1] normalised code is what NeonStatsEncoder is calibrated
 * against. */
constexpr float awbMaxChannel = 0.95f;

/* Minimum valid-patch count required to update the gains. With 256
 * patches a well-lit scene easily passes — only patches that are
 * neither saturated (maxCh > 0.95) nor at the noise floor
 * (minCh < awbMinChannel) count. Setting the floor at 96 (37.5 %
 * of the grid) keeps gray-world running on scenes that genuinely
 * have a representative neutral spread, but lets a dim shot — where
 * only a handful of patches escape the noise floor and those few
 * are typically a single bright object that is *not* gray —
 * fall through and hold the calibrated daylight prior. */
constexpr int awbMinValidPatches = 96;

/* Final clamp on the gain multipliers, relative to unity (G = 1.0).
 * Matches the [128, 1024] Q8 range the previous estimator used —
 * wide enough to correct for indoor incandescent / outdoor shade,
 * tight enough that a bad estimate on a monochrome scene can't
 * saturate one channel into nonsense. */
constexpr float awbGainMin = 0.5f;
constexpr float awbGainMax = 4.0f;

/* Knob derivation from the sensor's tuning. No silent fallbacks —
 * if a field is absent or zero, the corresponding member ends up
 * at zero, which naturally disables the matching path. */
float awbParam(const SensorTuning *t,
               float (SensorTuning::AwbParams::* field)) {
    return (t && (t->awbParams().*field) > 0.f) ? t->awbParams().*field : 0.f;
}

unsigned toQ8(float x) {
    return (unsigned)(x * 256.0f + 0.5f);
}

} /* namespace */

GrayWorldAwbController::GrayWorldAwbController(
        const SensorTuning *t,
        const float wbGainPrior[3])
    : tuning(t),
      /* Normalise the R / B priors against G so the shader-side WB,
       * which keeps G at unity, sees the same neutral. Falls back
       * to unity safely when the tuning has no CCT sets. */
      wbRPrior(wbGainPrior[1] > /*awbMinChannel*/ 0.02f
               ? wbGainPrior[0] / wbGainPrior[1] : 1.0f),
      wbBPrior(wbGainPrior[1] > 0.02f
               ? wbGainPrior[2] / wbGainPrior[1] : 1.0f),
      awbMinChannel(awbParam(t, &SensorTuning::AwbParams::cStatsMinThreshold)),
      awbDamping(awbParam(t, &SensorTuning::AwbParams::smoothingWpTrackingFraction)),
      current{wbRPrior, wbBPrior, 0} {
}

WbGains GrayWorldAwbController::currentGainsQ8() const {
    WbGains g;
    g.r = (uint16_t)toQ8(current.wbR);
    g.g = 256;
    g.b = (uint16_t)toQ8(current.wbB);
    return g;
}

void GrayWorldAwbController::reset() {
    current.wbR    = wbRPrior;
    current.wbB    = wbBPrior;
    current.estCct = 0;
}

AwbResult GrayWorldAwbController::process(const IpaStats &stats,
                                            float /*luxIndex*/,
                                            uint8_t /*awbMode*/) {
    AwbResult out;

    /* Gray-world over rgbMean patches, with saturated / near-black
     * patch exclusion. The pre-WB / pre-CCM domain means a clipped
     * patch skews disproportionately — filtering at the patch level
     * (not per-pixel, which we can't do here) is the usual
     * raw-domain AWB robustness step. */
    float sumR = 0.f, sumG = 0.f, sumB = 0.f;
    int   nValid = 0;
    for (int py = 0; py < IpaStats::PATCH_Y; ++py) {
        for (int px = 0; px < IpaStats::PATCH_X; ++px) {
            const float r = stats.rgbMean[py][px][0];
            const float g = stats.rgbMean[py][px][1];
            const float b = stats.rgbMean[py][px][2];
            const float maxCh = r > g ? (r > b ? r : b) : (g > b ? g : b);
            const float minCh = r < g ? (r < b ? r : b) : (g < b ? g : b);
            if (maxCh > awbMaxChannel) continue;
            if (minCh < awbMinChannel) continue;
            sumR += r; sumG += g; sumB += b;
            ++nValid;
        }
    }
    out.validPatchCount = nValid;

    if (nValid >= awbMinValidPatches) {
        const float meanR = sumR / (float)nValid;
        const float meanG = sumG / (float)nValid;
        const float meanB = sumB / (float)nValid;

        /* Gains boost each channel toward the G mean. Divide-by-
         * zero guarded by the awbMinChannel floor above (every
         * summed channel is at least awbMinChannel). */
        float rGain = meanG / meanR;
        float bGain = meanG / meanB;
        if (rGain < awbGainMin) rGain = awbGainMin;
        if (rGain > awbGainMax) rGain = awbGainMax;
        if (bGain < awbGainMin) bGain = awbGainMin;
        if (bGain > awbGainMax) bGain = awbGainMax;

        /* Always damp. The old "first-tick snap" skipped damping on
         * the very first valid tick and let lastWb jump from the
         * FusionLights prior straight to the current frame's gray-
         * world output — a visible WB shock even if the new state
         * was closer to correct. With the priors coming from a real
         * calibration anchor, the EMA at SmoothingWpTrackingFraction
         * = 0.1 crawls to the scene colour in ~20 frames with no
         * single-frame pops. */
        current.wbR = awbDamping * rGain + (1.0f - awbDamping) * current.wbR;
        current.wbB = awbDamping * bGain + (1.0f - awbDamping) * current.wbB;
    } else {
        /* Below the confidence gate: relax back toward the
         * calibrated daylight prior at the same damping the forward
         * path uses. Without this branch, a brief moment of
         * sufficient valid-patch coverage would land lastWb at a
         * possibly-biased gray-world reading and that bias would
         * freeze in place once the patch count fell back below the
         * gate — a "stuck cast". Symmetric pull-to-prior makes the
         * controller treat gate-failure scenes as low-confidence:
         * trust the sensor's calibrated neutral over a stale gray-
         * world estimate. */
        current.wbR = awbDamping * wbRPrior + (1.0f - awbDamping) * current.wbR;
        current.wbB = awbDamping * wbBPrior + (1.0f - awbDamping) * current.wbB;
    }

    /* Publish the smoothed gains. We always emit them, even when the
     * gate failed, so the coordinator's lock → unlock transition
     * resynchronises the shader to the controller's state. */
    WbGains gainsQ8;
    gainsQ8.r = (uint16_t)toQ8(current.wbR);
    gainsQ8.g = 256;
    gainsQ8.b = (uint16_t)toQ8(current.wbB);
    out.gains = gainsQ8;

    /* CCT-driven CCM. Convert the gray-world G/B ratio (= current.wbB
     * in our R-and-B-relative-to-G normalisation) into the NVIDIA
     * AWB-v4 chromaticity U = ln(G/B), pass through the sensor's
     * calibrated U→CCT fit, then LERP between the two CcmSets whose
     * cctK brackets the estimate. Requires the tuning to have the
     * awb.v4.{UtoCCT,CCTtoU,LowU,HighU} section; tunings that
     * predate it just skip the CCM update (caller's CCM buffer
     * keeps whatever the boot-time setup put there). */
    if (tuning && tuning->awbParams().loaded) {
        /* Apply the tuning's gray-line soft-clamp to U before
         * feeding CCT — noise on dark scenes pushes raw ln(G/B)
         * outside the calibrated range, and the clamp snaps it back
         * onto the locus the sensor was characterised over. */
        const float U  = tuning->clampU(logf(current.wbB));
        const int   K  = tuning->estimateCctFromU(U);
        current.estCct = K;
        out.estCct     = K;
        CcmQ10 ccm;
        tuning->ccmForCctLerpQ10(K, ccm.v);
        out.ccm = ccm;
    } else {
        out.estCct = current.estCct;
    }

    return out;
}

AwbResult GrayWorldAwbController::applyManualGains(
        float rGainAbs, float gGainAbs, float bGainAbs) {
    AwbResult out;

    const float gNorm = gGainAbs > 1e-3f ? gGainAbs : 1.0f;
    current.wbR = rGainAbs / gNorm;
    current.wbB = bGainAbs / gNorm;

    WbGains gainsQ8;
    gainsQ8.r = (uint16_t)toQ8(current.wbR);
    gainsQ8.g = 256;
    gainsQ8.b = (uint16_t)toQ8(current.wbB);
    out.gains  = gainsQ8;
    out.estCct = current.estCct;
    /* No CCM update — manual gains alone don't carry colour-matrix
     * intent; framework apps that want full manual colour set
     * COLOR_CORRECTION_TRANSFORM_MATRIX which we don't claim. */
    return out;
}

} /* namespace android */
