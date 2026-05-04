#include "BayesianAwbController.h"

#include <math.h>
#include <stdint.h>

#include <system/camera_metadata.h>

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
 * dead-band of any IIR damping we'll add later. fineSearch
 * refines off-grid in the perpendicular axis. */
constexpr int kCoarseSteps = 30;

/* fineSearch granularity. RPi default is 12; covers ±transverse
 * range with ~1/6th-step resolution. The off-curve search is
 * cheap (one cost eval per step), so granularity here is dwarfed
 * by the inner zone loop's runtime. */
constexpr int kFineSteps = 12;

/* Per-zone normalised chromaticity — R/G and B/G after the patch
 * filter. Lifted to anon-namespace so coarseSearch and fineSearch
 * share the storage shape, and the cost helper can take it by
 * pointer. */
struct Zone {
    float r;   /* R / G */
    float b;   /* B / G */
};

/* Cost-function inputs that don't change across the inner search
 * loops — the zone array, bias anchor, deltaLimit. Bundled here
 * so the helper signature stays sane and both coarseSearch /
 * fineSearch pass the same context. The lux-conditioned prior is
 * separate because fineSearch evaluates it at a fixed CT (the
 * coarseSearch winner) regardless of the off-curve offset. */
struct CostInputs {
    const Zone *zones;
    int         nValid;
    bool        hasBias;
    float       biasR;
    float       biasB;
    float       biasWeight;
    float       deltaLimit;
};

float computeCostAtGains(const CostInputs &c, float gainR, float gainB) {
    float err = 0.f;
    for (int z = 0; z < c.nValid; ++z) {
        const float dR = gainR * c.zones[z].r - 1.0f;
        const float dB = gainB * c.zones[z].b - 1.0f;
        float d2 = dR * dR + dB * dB;
        if (c.deltaLimit > 0.f && d2 > c.deltaLimit) d2 = c.deltaLimit;
        err += d2;
    }
    if (c.hasBias) {
        const float dR = gainR * c.biasR - 1.0f;
        const float dB = gainB * c.biasB - 1.0f;
        float d2 = dR * dR + dB * dB;
        if (c.deltaLimit > 0.f && d2 > c.deltaLimit) d2 = c.deltaLimit;
        err += d2 * c.biasWeight;
    }
    return err;
}

unsigned toQ8(float x) {
    return (unsigned)(x * 256.0f + 0.5f);
}

/* Lux-conditioned prior selection. Two priors bracketing the
 * current lux are blended linearly; outside the range, the closer
 * endpoint applies. With a single prior the blend reduces to that
 * one PWL at every CT step (luxIndex ignored), which is what test
 * tunings ship before the multi-illuminant calibration session. */
struct PriorBlend {
    const Pwl *lo;
    const Pwl *hi;
    float      blend;   /* 0 = pure lo, 1 = pure hi */
};

PriorBlend selectPriors(
        const std::vector<SensorTuning::BayesParams::LuxPrior> &priors,
        float luxIndex) {
    if (priors.empty())                       return {nullptr, nullptr, 0.f};
    if (priors.size() == 1)                   return {&priors[0].prior, nullptr, 0.f};
    if (luxIndex <= priors.front().lux)       return {&priors.front().prior, nullptr, 0.f};
    if (luxIndex >= priors.back().lux)        return {&priors.back().prior, nullptr, 0.f};
    for (size_t i = 1; i < priors.size(); ++i) {
        if (luxIndex <= priors[i].lux) {
            const float lo = priors[i - 1].lux;
            const float hi = priors[i].lux;
            return {&priors[i - 1].prior, &priors[i].prior,
                    (luxIndex - lo) / (hi - lo)};
        }
    }
    return {&priors.back().prior, nullptr, 0.f};
}

float evalPriorBlend(const PriorBlend &pb, float ct) {
    if (!pb.lo) return 0.f;
    const float yLo = pb.lo->eval(ct);
    if (!pb.hi) return yLo;
    const float yHi = pb.hi->eval(ct);
    return yLo + pb.blend * (yHi - yLo);
}

/* Camera2 AWB mode → tuning JSON key. The tuning's `bayes.modes`
 * is keyed by the strings emitted here so a regen of the tuning
 * doesn't have to hard-code Camera2 enum values. AUTO is mapped
 * but normally not present in the tuning (search uses the full
 * curve domain by default); a tuning *may* override it to clip
 * AUTO to a narrower band. Returns nullptr for unknown / OFF
 * modes — caller skips the mode clip. */
const char *awbModeToName(uint8_t awbMode) {
    switch (awbMode) {
    case ANDROID_CONTROL_AWB_MODE_AUTO:             return "AUTO";
    case ANDROID_CONTROL_AWB_MODE_INCANDESCENT:     return "INCANDESCENT";
    case ANDROID_CONTROL_AWB_MODE_FLUORESCENT:      return "FLUORESCENT";
    case ANDROID_CONTROL_AWB_MODE_WARM_FLUORESCENT: return "WARM_FLUORESCENT";
    case ANDROID_CONTROL_AWB_MODE_DAYLIGHT:         return "DAYLIGHT";
    case ANDROID_CONTROL_AWB_MODE_CLOUDY_DAYLIGHT:  return "CLOUDY_DAYLIGHT";
    case ANDROID_CONTROL_AWB_MODE_TWILIGHT:         return "TWILIGHT";
    case ANDROID_CONTROL_AWB_MODE_SHADE:            return "SHADE";
    default:                                        return nullptr;
    }
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
      frameCounter(0),
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
    frameCounter   = 0;
}

AwbResult BayesianAwbController::process(const IpaStats &stats,
                                          float luxIndex,
                                          uint8_t awbMode) {
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

    /* No usable signal — EMA-relax to the prior. Sharing the same
     * `damping` as the steady-state output smoothing means a
     * temporary drop in valid zones doesn't snap WB; gains crawl
     * back toward the calibrated daylight neutral over the same
     * ~20-frame window. damping == 0 falls back to a hard reset. */
    if (nValid == 0) {
        const float dampNoSig = bayes->damping;
        if (dampNoSig > 0.f) {
            current.wbR = dampNoSig * wbRPrior + (1.0f - dampNoSig) * current.wbR;
            current.wbB = dampNoSig * wbBPrior + (1.0f - dampNoSig) * current.wbB;
        } else {
            current.wbR = wbRPrior;
            current.wbB = wbBPrior;
        }
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
    float ctMin  = ctRMin > ctBMin ? ctRMin : ctBMin;
    float ctMax  = ctRMax < ctBMax ? ctRMax : ctBMax;

    /* Manual-preset clip — Camera2 AWB modes other than AUTO / OFF
     * narrow the search to a calibrated CT band so the resulting
     * gains land near a known illuminant regardless of what the
     * scene actually looks like. Same coarseSearch + fineSearch
     * pipeline; only the [ctMin, ctMax] window changes. Modes
     * absent from the tuning's `modes` map fall through to the
     * full search range — the controller doesn't fail on an app's
     * unrecognised preset. */
    const char *modeName = awbModeToName(awbMode);
    if (modeName) {
        for (size_t i = 0; i < bayes->modes.size(); ++i) {
            if (bayes->modes[i].first == modeName) {
                const float lo = bayes->modes[i].second.ctLo;
                const float hi = bayes->modes[i].second.ctHi;
                if (lo > ctMin) ctMin = lo;
                if (hi < ctMax) ctMax = hi;
                break;
            }
        }
    }
    /* Empty-range guard — clip can collapse to a single point if
     * the preset's range falls outside the calibrated curves;
     * coarseSearch handles equal endpoints by re-evaluating the
     * same t every step, but ctMin > ctMax would break logf. */
    if (ctMin > ctMax) ctMax = ctMin;

    /* coarseSearch — log-stepped traversal of the calibrated CT
     * range. Cost at each candidate t:
     *   `Σ_zones min((gainR·R/G − 1)² + (gainB·B/G − 1)², deltaLimit)
     *      − prior(t)`
     * `deltaLimit` caps how much one off-grey zone can pull the
     * sum (a saturated-blue laptop screen or a green carpet would
     * otherwise dominate the squared-error landscape regardless of
     * the rest of the scene). `prior(t)` is the lux-conditioned
     * tuning anchor: subtracted from the cost so higher prior →
     * preferred t. Both default to inert (deltaLimit = 0 → no
     * clamp; empty/uniform priors → contribution zero), which is
     * the Step-5-equivalent behaviour and the path tunings without
     * a calibrated bayes block fall down. */
    const float logMin     = logf(ctMin);
    const float logMax     = logf(ctMax);
    const float deltaLimit = bayes->deltaLimit;
    const PriorBlend prior = selectPriors(bayes->priors, luxIndex);

    /* Bias samples — synthetic zone anchored at `biasCT` with
     * weight `biasProportion × nValid`. See Step 7a commit.
     * Disabled when `biasCT` or `biasProportion` is zero. */
    CostInputs cost;
    cost.zones      = zones;
    cost.nValid     = nValid;
    cost.hasBias    = bayes->biasCT > 0.f && bayes->biasProportion > 0.f;
    cost.biasR      = cost.hasBias ? bayes->ctCurveR.eval(bayes->biasCT) : 0.f;
    cost.biasB      = cost.hasBias ? bayes->ctCurveB.eval(bayes->biasCT) : 0.f;
    cost.biasWeight = cost.hasBias ? bayes->biasProportion * (float)nValid : 0.f;
    cost.deltaLimit = deltaLimit;

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

        const float err = computeCostAtGains(cost, gainR, gainB)
                        - evalPriorBlend(prior, t);
        if (!bestSet || err < bestErr) {
            bestErr = err;
            bestT   = t;
            bestSet = true;
        }
    }

    /* fineSearch — refine the on-curve coarseSearch winner along
     * the axis perpendicular to the CT curve in (R/G, B/G) space.
     * The on-curve search alone can only express CT shifts; real
     * scenes also need magenta ↔ green correction (off-curve in
     * Planckian terms — fluorescent banks, mixed light, sensor
     * IR-leak). transversePos / transverseNeg cap the displacement
     * on each side of the curve in chromaticity units (RPi
     * defaults ≈ 0.03, i.e. ~3 % off-curve). The prior contribution
     * is constant across this loop — fineSearch doesn't change CT
     * — so it falls out of the argmin and we skip subtracting it. */
    /* On-curve anchor — fixed throughout fineSearch. The winner
     * (bestR / bestB) is updated as candidates beat it but never
     * feeds back into candidate generation; if we let it, sequential
     * wins compound the offset and the search drifts well past the
     * transverse caps. */
    const float baseR = bayes->ctCurveR.eval(bestT);
    const float baseB = bayes->ctCurveB.eval(bestT);
    float bestR = baseR;
    float bestB = baseB;
    const bool fineEnabled = bayes->transversePos > 0.f
                          || bayes->transverseNeg > 0.f;
    if (fineEnabled) {
        /* Tangent at bestT via centred finite difference; perpendicular
         * is the 90°-rotated unit tangent. ε at 1 % of CT keeps the
         * sample inside the calibrated curve domain. */
        const float dt   = bestT * 0.01f;
        const float dr   = bayes->ctCurveR.eval(bestT + dt)
                         - bayes->ctCurveR.eval(bestT - dt);
        const float db   = bayes->ctCurveB.eval(bestT + dt)
                         - bayes->ctCurveB.eval(bestT - dt);
        const float norm = sqrtf(dr * dr + db * db);
        if (norm > 1e-6f) {
            const float perpR = -db / norm;
            const float perpB =  dr / norm;
            float bestFineErr = computeCostAtGains(cost,
                                                    1.0f / baseR,
                                                    1.0f / baseB);
            const float tPos = bayes->transversePos;
            const float tNeg = bayes->transverseNeg;
            for (int i = 0; i < kFineSteps; ++i) {
                const float frac   = (float)i / (float)(kFineSteps - 1);
                const float offset = -tNeg + frac * (tPos + tNeg);
                const float candR  = baseR + offset * perpR;
                const float candB  = baseB + offset * perpB;
                if (candR < 1e-6f || candB < 1e-6f) continue;
                const float err = computeCostAtGains(cost,
                                                      1.0f / candR,
                                                      1.0f / candB);
                if (err < bestFineErr) {
                    bestFineErr = err;
                    bestR       = candR;
                    bestB       = candB;
                }
            }
        }
    }

    /* Temporal smoothing on the published gains. First
     * `startupFrames` after reset bypass the IIR (hard snap) so
     * cold-start lands on the bayes estimate immediately instead
     * of crawling from the calibrated prior. Past that, the EMA
     * at `damping` rate dampens both per-tick search noise and
     * scene-change transients. damping == 0 disables smoothing —
     * gains track the per-tick estimate verbatim, which is the
     * Step-7 behaviour. estCct is reported from coarseSearch
     * directly; the off-curve fineSearch displaces gains in
     * chromaticity, not in CT. */
    const float gainRTarget = 1.0f / bestR;
    const float gainBTarget = 1.0f / bestB;

    float speed = 1.0f;
    if (frameCounter >= (uint32_t)bayes->startupFrames && bayes->damping > 0.f)
        speed = bayes->damping;

    current.wbR    = speed * gainRTarget + (1.0f - speed) * current.wbR;
    current.wbB    = speed * gainBTarget + (1.0f - speed) * current.wbB;
    current.estCct = (int)(bestT + 0.5f);
    if (frameCounter < UINT32_MAX) ++frameCounter;

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
