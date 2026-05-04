#include "BayesianAwbController.h"

#include <math.h>
#include <stdint.h>

#include "ipa/IpaStats.h"

namespace android {

namespace {

/* Same per-patch saturation / noise-floor filter the gray-world
 * controller uses. Patches with any channel above the saturation
 * cap are dropped (the missing highlight skews the chromaticity);
 * patches with any channel below the sensor's noise floor are
 * dropped (no reliable colour signal). Identical thresholds keep
 * the two AWB impls comparable on the same scene. */
constexpr float awbMaxChannel = 0.95f;

/* coarseSearch granularity. Log-stepped traversal from ctMin to
 * ctMax; 30 points across the typical 2000-10000 K calibration
 * range gives ~140 K resolution near 4000 K — finer than the
 * dead-band of any IIR damping we'll add later. fineSearch (step
 * 7) refines off-grid. */
constexpr int kCoarseSteps = 30;

unsigned toQ8(float x) {
    return (unsigned)(x * 256.0f + 0.5f);
}

} /* namespace */

BayesianAwbController::BayesianAwbController(const SensorTuning *t,
                                              const float wbGainPrior[3])
    : tuning(t),
      bayes(t && t->bayesParams() ? &(*t->bayesParams()) : nullptr),
      wbRPrior(wbGainPrior[1] > 0.02f
               ? wbGainPrior[0] / wbGainPrior[1] : 1.0f),
      wbBPrior(wbGainPrior[1] > 0.02f
               ? wbGainPrior[2] / wbGainPrior[1] : 1.0f),
      current{wbRPrior, wbBPrior, 0} {
}

WbGains BayesianAwbController::currentGainsQ8() const {
    WbGains g;
    g.r = (uint16_t)toQ8(current.wbR);
    g.g = 256;
    g.b = (uint16_t)toQ8(current.wbB);
    return g;
}

void BayesianAwbController::reset() {
    current.wbR    = wbRPrior;
    current.wbB    = wbBPrior;
    current.estCct = 0;
}

AwbResult BayesianAwbController::process(const IpaStats &stats,
                                          float /*luxIndex*/) {
    AwbResult out;

    /* Defensive — the AwbFactory only routes us when bayesParams
     * is engaged, but if that invariant ever loosens, fall through
     * to publishing the prior. Same shape as the no-zones branch
     * below; both keep `current` unchanged so the next valid tick
     * resumes from the prior, not a stale estimate. */
    if (!bayes) {
        WbGains gainsQ8;
        gainsQ8.r = (uint16_t)toQ8(current.wbR);
        gainsQ8.g = 256;
        gainsQ8.b = (uint16_t)toQ8(current.wbB);
        out.gains  = gainsQ8;
        out.estCct = current.estCct;
        return out;
    }

    /* Per-zone normalise R/G and B/G after the same patch filter
     * gray-world uses. Storing zones up-front instead of recomputing
     * per CT step keeps the inner search loop tight; with 256 patches
     * × 30 CT steps × ~10 ops we sit well under 0.1 ms even on
     * Cortex-A15 scalar. */
    const float awbMinChannel = tuning
        ? tuning->awbParams().cStatsMinThreshold : 0.f;
    struct Zone {
        float r;   /* R / G */
        float b;   /* B / G */
    };
    Zone zones[IpaStats::PATCH_X * IpaStats::PATCH_Y];
    int  nValid = 0;
    for (int py = 0; py < IpaStats::PATCH_Y; ++py) {
        for (int px = 0; px < IpaStats::PATCH_X; ++px) {
            const float r = stats.rgbMean[py][px][0];
            const float g = stats.rgbMean[py][px][1];
            const float b = stats.rgbMean[py][px][2];
            const float maxCh = r > g ? (r > b ? r : b) : (g > b ? g : b);
            const float minCh = r < g ? (r < b ? r : b) : (g < b ? g : b);
            if (maxCh > awbMaxChannel) continue;
            if (minCh < awbMinChannel) continue;
            /* +1 epsilon guards against zero G — patch already
             * passed the noise floor, so g is at least awbMinChannel
             * but the divider is cheap insurance. */
            const float gNorm = g > 1e-6f ? g : 1e-6f;
            zones[nValid].r = r / gNorm;
            zones[nValid].b = b / gNorm;
            ++nValid;
        }
    }
    out.validPatchCount = nValid;

    /* No usable signal — relax to prior. No EMA yet; the controller
     * just publishes the prior verbatim. */
    if (nValid == 0) {
        current.wbR = wbRPrior;
        current.wbB = wbBPrior;
        WbGains gainsQ8;
        gainsQ8.r = (uint16_t)toQ8(current.wbR);
        gainsQ8.g = 256;
        gainsQ8.b = (uint16_t)toQ8(current.wbB);
        out.gains  = gainsQ8;
        out.estCct = current.estCct;
        return out;
    }

    /* Search domain — intersection of the two CT curves' x ranges.
     * If either is degenerate (single calibration point) the search
     * collapses to that single point. */
    const float ctRMin = bayes->ctCurveR.minX();
    const float ctRMax = bayes->ctCurveR.maxX();
    const float ctBMin = bayes->ctCurveB.minX();
    const float ctBMax = bayes->ctCurveB.maxX();
    const float ctMin  = ctRMin > ctBMin ? ctRMin : ctBMin;
    const float ctMax  = ctRMax < ctBMax ? ctRMax : ctBMax;

    /* coarseSearch — log-stepped traversal of the calibrated CT
     * range. Likelihood = sum over zones of `(gainR·R/G − 1)² +
     * (gainB·B/G − 1)²`. No prior, no clamp, no off-curve
     * refinement on this step. */
    const float logMin = logf(ctMin);
    const float logMax = logf(ctMax);
    float bestT     = ctMin;
    float bestErr   = 0.f;
    bool  bestSet   = false;
    for (int i = 0; i < kCoarseSteps; ++i) {
        const float frac  = (float)i / (float)(kCoarseSteps - 1);
        const float logCt = logMin + frac * (logMax - logMin);
        const float t     = expf(logCt);

        const float ctR = bayes->ctCurveR.eval(t);
        const float ctB = bayes->ctCurveB.eval(t);
        if (ctR < 1e-6f || ctB < 1e-6f) continue;
        const float gainR = 1.0f / ctR;
        const float gainB = 1.0f / ctB;

        float err = 0.f;
        for (int z = 0; z < nValid; ++z) {
            const float dR = gainR * zones[z].r - 1.0f;
            const float dB = gainB * zones[z].b - 1.0f;
            err += dR * dR + dB * dB;
        }
        if (!bestSet || err < bestErr) {
            bestErr = err;
            bestT   = t;
            bestSet = true;
        }
    }

    /* Convert the winning CT back to candidate gains and publish.
     * No IIR smoothing yet — output is jittery on real frames; that
     * gets fixed in step 8. */
    const float bestGainR = 1.0f / bayes->ctCurveR.eval(bestT);
    const float bestGainB = 1.0f / bayes->ctCurveB.eval(bestT);
    current.wbR    = bestGainR;
    current.wbB    = bestGainB;
    current.estCct = (int)(bestT + 0.5f);

    WbGains gainsQ8;
    gainsQ8.r = (uint16_t)toQ8(current.wbR);
    gainsQ8.g = 256;
    gainsQ8.b = (uint16_t)toQ8(current.wbB);
    out.gains  = gainsQ8;
    out.estCct = current.estCct;
    return out;
}

AwbResult BayesianAwbController::applyManualGains(
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
    return out;
}

} /* namespace android */
