#define LOG_TAG "Cam-AF"

#include "AutoFocusController.h"

#include <math.h>

#include <utils/Log.h>
#include <system/camera_metadata.h>

#include "V4l2Device.h"
#include "IspPipeline.h"
#include "ipa/Ipa.h"
#include "sensor/SensorTuning.h"

namespace android {

namespace {

/* VCM positions are raw units exposed by the focuser subdev. The map
 * to diopters is linear within the usable range; 50 units per diopter.
 * Values outside [kVcmMin, kVcmMax] are rejected by the driver. The
 * inf / macro / settle-frames values come from SensorTuning when
 * present; these are fallbacks for tuning-less builds. */
constexpr int32_t kVcmInfinityFallback = 140;
constexpr int32_t kVcmMacroEndFallback = 650;
constexpr int32_t kVcmAutoEndFallback  = 640;
constexpr int32_t kVcmMin              = 0;
constexpr int32_t kVcmMax              = 1023;
constexpr float   kVcmPerDiopter       = 50.0f;

/* Centre region of the patch grid: the inclusive [4, 11] band on each
 * axis covers the middle 50 % of the frame on both dimensions —
 * matches what the previous RGBA-domain sharpness measured on the
 * centre 1/4 of the rendered preview. */
constexpr int kRoiPatchLo = 4;
constexpr int kRoiPatchHi = 12;

/* Compile-time CDAF defaults — used only when the tuning is missing
 * the matching `active.af.*` keys. Same magnitudes the JSON ships
 * for the IMX179 module. */
constexpr int32_t kDefStepCoarse     = 25;
constexpr int32_t kDefStepFine       = 5;
constexpr float   kDefContrastRatio  = 0.75f;
constexpr float   kDefRetriggerRatio = 0.75f;
constexpr int32_t kDefRetriggerDelay = 10;

float sumCentreFocus(
    const float metric[IpaStats::PATCH_Y][IpaStats::PATCH_X]) {
    float s = 0.f;
    for (int py = kRoiPatchLo; py < kRoiPatchHi; ++py) {
        for (int px = kRoiPatchLo; px < kRoiPatchHi; ++px) {
            s += metric[py][px];
        }
    }
    return s;
}

void sumCentreRgb(
    const float rgbMean[IpaStats::PATCH_Y][IpaStats::PATCH_X][3],
    float out[3]) {
    out[0] = out[1] = out[2] = 0.f;
    for (int py = kRoiPatchLo; py < kRoiPatchHi; ++py) {
        for (int px = kRoiPatchLo; px < kRoiPatchHi; ++px) {
            out[0] += rgbMean[py][px][0];
            out[1] += rgbMean[py][px][1];
            out[2] += rgbMean[py][px][2];
        }
    }
}

bool outsideRatio(float a, float b, float ratio) {
    /* Symmetric ratio test with a small tolerance to absorb sensor
     * noise — `+1.0` matches the magnitude RPi libcamera uses on
     * its (much smaller) raw stats values. Both `a / b < ratio` and
     * `b / a < ratio` register as "different enough"; either side
     * darkening or brightening counts. */
    return (a + 1.f) < ratio * b || (b + 1.f) < ratio * a;
}

int32_t clampVcm(int32_t pos) {
    if (pos < kVcmMin) return kVcmMin;
    if (pos > kVcmMax) return kVcmMax;
    return pos;
}

} /* namespace */

AutoFocusController::AutoFocusController(V4l2Device *dev, IspPipeline *isp,
                                         Ipa *ipa,
                                         const SensorTuning *tuning)
    : mDev(dev)
    , mIsp(isp)
    , mIpa(ipa)
    , mVcmInfinity(kVcmInfinityFallback)
    , mVcmMacroStart(0)
    , mVcmMacroEnd(kVcmMacroEndFallback)
    , mVcmAutoEnd(kVcmAutoEndFallback)
    , mStepCoarse(kDefStepCoarse)
    , mStepFine(kDefStepFine)
    , mContrastRatio(kDefContrastRatio)
    , mRetriggerRatio(kDefRetriggerRatio)
    , mRetriggerDelay(kDefRetriggerDelay)
    , mPdafEnabled(false)
    , mSettleFramesCoarse(2)
    , mSettleFramesFine(1)
    , mAfMode(ANDROID_CONTROL_AF_MODE_OFF)
    , mFocusPosition(0)
    , mPreSweepFocusPosition(0)
    , mState(ScanState::Idle)
    , mSweepPos(0)
    , mSweepStep(0)
    , mSweepBestPos(0)
    , mSweepBestScore(0.f)
    , mSettleFrames(0)
    , mCoarsePhaseStart(0)
    , mFinePhaseStart(0)
    , mCoarseReversed(false)    , mSceneFocusSnapshot(0.f)
    , mSceneRgbSnapshot{0.f, 0.f, 0.f}
    , mSceneChangeCount(0)
    , mFramesSinceLastSweep(UINT32_MAX)
{
    if (tuning && tuning->isLoaded() && tuning->hasAf()) {
        const SensorTuning::AfParams &af = tuning->af();
        if (af.moduleCalEnable) {
            mVcmInfinity = af.infPos + af.infOffset;
            mVcmMacroEnd = af.macroPos + af.macroOffset;
        } else {
            mVcmInfinity = af.infPos;
            mVcmMacroEnd = af.macroPos;
        }
        mVcmAutoEnd = af.macroPos;

        if (af.stepCoarse         > 0)   mStepCoarse         = af.stepCoarse;
        if (af.stepFine           > 0)   mStepFine           = af.stepFine;
        if (af.contrastRatio      > 0.f) mContrastRatio      = af.contrastRatio;
        if (af.retriggerRatio     > 0.f) mRetriggerRatio     = af.retriggerRatio;
        if (af.retriggerDelay     > 0)   mRetriggerDelay     = af.retriggerDelay;
        if (af.settleFramesCoarse > 0)   mSettleFramesCoarse = af.settleFramesCoarse;
        if (af.settleFramesFine   > 0)   mSettleFramesFine   = af.settleFramesFine;
        mPdafEnabled = af.pdafEnabled;

        ALOGD("AF tuning: cal=%d inf=%d macro=%d auto_end=%d "
              "step=%d/%d settle=%d/%d ratios=%.2f/%.2f delay=%d pdaf=%d",
              af.moduleCalEnable ? 1 : 0,
              mVcmInfinity, mVcmMacroEnd, mVcmAutoEnd,
              mStepCoarse, mStepFine,
              mSettleFramesCoarse, mSettleFramesFine,
              (double)mContrastRatio, (double)mRetriggerRatio, mRetriggerDelay,
              mPdafEnabled ? 1 : 0);

        if (mPdafEnabled) {
            ALOGW("AF: pdaf_enabled=true but no PDAF state-machine wired; "
                  "falling back to CDAF only");
        }
    }
    /* Macro mode starts at the midpoint between infinity and macro. */
    mVcmMacroStart = (mVcmInfinity + mVcmMacroEnd) / 2;

    mFocusPosition = mVcmInfinity;
    mSweepBestPos  = mVcmInfinity;
}

bool AutoFocusController::nearLimit(int32_t pos, int32_t limit) const {
    /* "Near" = within two coarse steps. Beyond that the sweep would
     * almost immediately bump into the limit and have no room to
     * find the peak; better to start from the limit and step toward
     * the centre. */
    int32_t diff = pos - limit;
    if (diff < 0) diff = -diff;
    return diff <= 2 * mStepCoarse;
}

void AutoFocusController::beginCoarseFromCurrent(uint8_t afMode) {
    /* AUTO (and any non-continuous mode) always sweeps the full
     * range from focusMin forward — no bidirectional logic, no
     * Coarse1 → reverse → Coarse2 dance. The user explicitly tapped
     * to focus, so we pay ~25-30 frames of latency in exchange for
     * never picking the wrong direction. RPi libcamera ships the
     * same `mode_ != AfModeContinuous → start at focusMin` clause.
     *
     * CONTINUOUS_PICTURE keeps the bidirectional logic: scene-change
     * retriggers fire often enough that a full-range scan every
     * time would be visibly disruptive, and the directional probe
     * + reversal-on-failure handles the typical small-correction
     * case quickly. */
    const bool continuous =
        afMode == ANDROID_CONTROL_AF_MODE_CONTINUOUS_PICTURE;

    if (!continuous) {
        mSweepPos       = mVcmInfinity;
        mSweepStep      = mStepCoarse;
        mState          = ScanState::Coarse2;
        mCoarseReversed = true;
    } else if (nearLimit(mFocusPosition, mVcmInfinity)) {
        mSweepPos       = mVcmInfinity;
        mSweepStep      = mStepCoarse;
        mState          = ScanState::Coarse2;
        mCoarseReversed = true;
    } else if (nearLimit(mFocusPosition, mVcmAutoEnd)) {
        mSweepPos       = mVcmAutoEnd;
        mSweepStep      = -mStepCoarse;
        mState          = ScanState::Coarse2;
        mCoarseReversed = true;
    } else {
        mSweepPos       = mFocusPosition;
        mSweepStep      = -mStepCoarse;
        mState          = ScanState::Coarse1;
        mCoarseReversed = false;
    }
    mSweepPos = clampVcm(mSweepPos);
}

void AutoFocusController::startSweep(uint8_t afMode) {
    /* Snapshot the converged lens position so a Failed sweep can
     * park the lens here instead of committing the noise argmax
     * of a flat scan. */
    mPreSweepFocusPosition = mFocusPosition;
    mIsp->setAwbLock(true);
    /* Hold the converged AE target across the sweep. The score is
     * already exposure-invariant by construction (focusMetric =
     * grad²/pixel²), but locking AE removes any residual brightness
     * drift from sensor-side gain ladders and stops the AE
     * controller from chasing scene changes the sweep itself
     * provokes. Released on commit / cancel / reset. */
    if (mIpa) mIpa->setAeLock(true);
    mSettleFrames     = 0;
    mSweepBestScore   = 0.f;
    mScanData.clear();
    mCoarsePhaseStart = 0;
    mFinePhaseStart   = 0;
    mSceneChangeCount = 0;

    if (afMode == ANDROID_CONTROL_AF_MODE_MACRO) {
        /* Macro is a half-range scan from the midpoint to the macro
         * end. Skip the directional probe — we already know which
         * way the lens has to go. */
        mSweepPos       = mVcmMacroStart;
        mSweepStep      = mStepCoarse;
        mState          = ScanState::Coarse2;
        mCoarseReversed = true;
        mSweepBestPos   = mSweepPos;
    } else {
        beginCoarseFromCurrent(afMode);
        mSweepBestPos   = mSweepPos;
    }

    ALOGD("AF sweep started (mode=%u, start=%d step=%d state=%s)",
          afMode, mSweepPos, mSweepStep,
          mState == ScanState::Coarse1 ? "Coarse1" : "Coarse2");
}

void AutoFocusController::cancelSweep() {
    if (mState == ScanState::Idle) return;
    mIsp->setAwbLock(false);
    if (mIpa) mIpa->setAeLock(false);
    mState = ScanState::Idle;
}

void AutoFocusController::recordSample(int32_t pos, float score) {
    mScanData.push_back({pos, score});
    if (score > mSweepBestScore) {
        mSweepBestScore = score;
        mSweepBestPos   = pos;
    }
}

int32_t AutoFocusController::findPeak(size_t start, size_t end) const {
    if (end > mScanData.size()) end = mScanData.size();
    if (start >= end) return mSweepBestPos;

    /* Discrete argmax across the slice. */
    size_t maxIdx = start;
    for (size_t i = start + 1; i < end; ++i) {
        if (mScanData[i].score > mScanData[maxIdx].score) maxIdx = i;
    }
    /* No room to fit a parabola: return the discrete winner. */
    if (maxIdx == start || maxIdx + 1 >= end) {
        return mScanData[maxIdx].pos;
    }

    const ScanSample &s0 = mScanData[maxIdx - 1];
    const ScanSample &s1 = mScanData[maxIdx];
    const ScanSample &s2 = mScanData[maxIdx + 1];
    /* Parabolic vertex through (s0, s1, s2) with s1 the discrete
     * max. Numerically stable form, factoring out common diffs.
     * Degenerate denominator (collinear points) → discrete max. */
    const float x0 = (float)s0.pos, y0 = s0.score;
    const float x1 = (float)s1.pos, y1 = s1.score;
    const float x2 = (float)s2.pos, y2 = s2.score;
    const float a = (x1 - x0);
    const float b = (x1 - x2);
    const float p = a * (y1 - y2);
    const float q = b * (y1 - y0);
    const float denom = p - q;
    if (fabsf(denom) < 1e-6f) return s1.pos;
    const float vx = x1 - 0.5f * (a * p - b * q) / denom;
    int32_t out = (int32_t)(vx + 0.5f);
    /* Clamp to the outer samples — a degenerate fit can extrapolate
     * well outside the immediate bracket. */
    int32_t lo = s0.pos < s2.pos ? s0.pos : s2.pos;
    int32_t hi = s0.pos < s2.pos ? s2.pos : s0.pos;
    if (out < lo) out = lo;
    if (out > hi) out = hi;
    return clampVcm(out);
}

void AutoFocusController::advanceCoarse(float score) {
    const int32_t prevPos = mSweepPos;
    recordSample(mSweepPos, score);

    /* How many Coarse-phase samples have advanced the running max.
     * `>= 2` means the curve has actually risen at least once in
     * this direction, which is the necessary precondition for
     * peakPassed to mean anything. Without it, a single dip below
     * `ratio × first_sample` would trip Fine on a phase that
     * hasn't done any exploration. Counted over the slice
     * `[mCoarsePhaseStart, end)` so a Coarse1→Coarse2 reversal-
     * with-reset doesn't carry the stale Coarse1 ladder forward. */
    size_t phaseRising = 0;
    float  phaseMax    = 0.f;
    for (size_t i = mCoarsePhaseStart; i < mScanData.size(); ++i) {
        if (mScanData[i].score > phaseMax) {
            phaseMax = mScanData[i].score;
            phaseRising++;
        }
    }
    const size_t phaseSampleCount = mScanData.size() - mCoarsePhaseStart;

    const bool peakPassed =
        mSweepBestScore > 0.f &&
        score < mContrastRatio * mSweepBestScore &&
        (mState == ScanState::Coarse1 || phaseRising >= 2);
    const bool atLimit =
        (mSweepStep > 0 && mSweepPos >= mVcmAutoEnd) ||
        (mSweepStep < 0 && mSweepPos <= mVcmInfinity);

    /* Coarse1 fast-path: one big step downwards on the second
     * sample → reverse without walking the whole half-range. */
    const bool fastReverse =
        mState == ScanState::Coarse1 && !mCoarseReversed &&
        phaseSampleCount == 2 &&
        score < mContrastRatio * mSweepBestScore;

    /* Slow-descent reversal: Coarse1 walked all the way to the
     * peakPassed / atLimit gate but the running max never moved
     * past the start sample (`phaseRising <= 1`). Means the curve
     * slid downward the whole way — the peak is on the opposite
     * side. RPi libcamera uses the same first-sample-was-max
     * check after Coarse1 termination. */
    const bool slowReverse =
        mState == ScanState::Coarse1 && !mCoarseReversed &&
        (peakPassed || atLimit) &&
        phaseRising <= 1;

    if (fastReverse || slowReverse) {
        mSweepStep      = -mSweepStep;
        mState          = ScanState::Coarse2;
        mCoarseReversed = true;
        mSweepPos       = mSweepBestPos;
        /* Coarse1's running max wasn't a real peak — that's the
         * trigger condition. Drop the running peak score so
         * Coarse2's peakPassed gate compares against its own
         * fresh max; bump the phase boundary so subsequent
         * findPeak() / phase-rising counts only see Coarse2. The
         * scan history itself stays intact for diagnostics. */
        mSweepBestScore   = 0.f;
        mCoarsePhaseStart = mScanData.size();
    } else if (peakPassed || atLimit) {
        /* Coarse → Fine handoff. Parabolic peak over the Coarse
         * phase samples gives a sub-step centre for the Fine
         * bracket — even when the real maximum sits between two
         * coarse-step anchors, the Fine sweep can land on it. */
        const int32_t coarsePk = findPeak(mCoarsePhaseStart,
                                           mScanData.size());
        if (mSweepStep > 0) {
            mSweepPos  = clampVcm(coarsePk - mStepFine);
            mSweepStep = mStepFine;
        } else {
            mSweepPos  = clampVcm(coarsePk + mStepFine);
            mSweepStep = -mStepFine;
        }
        mState          = ScanState::Fine;
        mFinePhaseStart = mScanData.size();
    } else {
        mSweepPos = clampVcm(mSweepPos + mSweepStep);
    }

    setSettleForMove(prevPos, mSweepPos);
}

void AutoFocusController::advanceFine(float score) {
    const int32_t prevPos = mSweepPos;
    recordSample(mSweepPos, score);

    /* Three Fine samples bracket the peak well enough to interpolate
     * a sub-stepFine maximum; past that we'd just be fighting VCM
     * noise. findPeak over the Fine slice picks discrete argmax +
     * parabolic on its immediate neighbours, so the commit position
     * benefits from sub-step accuracy whenever the actual maximum
     * sits between two fine-step samples. */
    if (mScanData.size() - mFinePhaseStart >= 3) {
        const int32_t finePk = findPeak(mFinePhaseStart,
                                         mScanData.size());
        mSweepPos = finePk;
        mState    = ScanState::Settle;
    } else {
        mSweepPos = clampVcm(mSweepPos + mSweepStep);
    }

    setSettleForMove(prevPos, mSweepPos);
}

void AutoFocusController::setSettleForMove(int32_t fromPos, int32_t toPos) {
    /* Pick settle frames from the magnitude of the actual VCM move,
     * not from the state we're about to be in. Coarse-magnitude
     * moves (initial probe steps, direction reversal back to the
     * running peak, the jump from coarse position into the Fine
     * bracket) all need the longer wait to let the open-loop motor
     * physically arrive — there's no V4L2 feedback on actual lens
     * position, so we wait by frame count instead. Fine-magnitude
     * moves take the shorter wait. */
    const int32_t mag = (toPos > fromPos) ? (toPos - fromPos)
                                          : (fromPos - toPos);
    mSettleFrames = (mag > mStepFine) ? mSettleFramesCoarse
                                      : mSettleFramesFine;
}

void AutoFocusController::commitSweep(bool focused) {
    /* On Failed, park back at the position the lens was at before
     * this sweep started — committing the noise-driven argmax of a
     * flat scan actively *defocuses* the image relative to whatever
     * the previous sweep had landed on. On Focused, drive to the
     * Fine-interpolated peak (mSweepPos at this point — set by
     * advanceFine to the parabolic vertex). Falls back to
     * mSweepBestPos only if Fine never managed a fit, which happens
     * when Coarse hit a range limit before bracketing the peak. */
    int32_t finalPos;
    if (!focused) {
        finalPos = mPreSweepFocusPosition;
    } else {
        const size_t fineSamples = mScanData.size() - mFinePhaseStart;
        finalPos = (fineSamples >= 3) ? mSweepPos : mSweepBestPos;
    }
    mDev->setFocusPosition(finalPos);
    mFocusPosition = finalPos;
    mIsp->setAwbLock(false);
    if (mIpa) mIpa->setAeLock(false);
    /* Force the next idle onStats to capture a fresh scene snapshot
     * — at commit time we don't yet have a stats frame for the
     * just-committed lens position; better to read it on the next
     * incoming frame than to copy stale running values. */
    mSceneFocusSnapshot = 0.f;
    mSceneRgbSnapshot[0]    = 0.f;
    mSceneRgbSnapshot[1]    = 0.f;
    mSceneRgbSnapshot[2]    = 0.f;
    mSceneChangeCount   = 0;
    mFramesSinceLastSweep = 0;
    mState = ScanState::Idle;

    ALOGD("AF done: pos=%d (best=%d) score=%.0f result=%s",
          finalPos, mSweepBestPos, (double)mSweepBestScore,
          focused ? "Focused" : "Failed");
}

void AutoFocusController::onSettings(const CameraMetadata &cm,
                                     uint32_t /*frameNumber*/) {
    uint8_t afMode = ANDROID_CONTROL_AF_MODE_OFF;
    if (cm.exists(ANDROID_CONTROL_AF_MODE))
        afMode = *cm.find(ANDROID_CONTROL_AF_MODE).data.u8;

    if (afMode == ANDROID_CONTROL_AF_MODE_OFF && mState != ScanState::Idle)
        cancelSweep();
    mAfMode = afMode;

    /* Manual focus: LENS_FOCUS_DISTANCE (diopters) -> VCM. */
    if (afMode == ANDROID_CONTROL_AF_MODE_OFF &&
        cm.exists(ANDROID_LENS_FOCUS_DISTANCE)) {
        float diopter = *cm.find(ANDROID_LENS_FOCUS_DISTANCE).data.f;
        int32_t pos = mVcmInfinity + (int32_t)(diopter * kVcmPerDiopter);
        pos = clampVcm(pos);
        mDev->setFocusPosition(pos);
        mFocusPosition = pos;
    }

    /* AF_TRIGGER_START kicks off a sweep. Only valid when AF_MODE
     * is AUTO / MACRO / CONTINUOUS_PICTURE. */
    if (cm.exists(ANDROID_CONTROL_AF_TRIGGER)) {
        uint8_t trigger = *cm.find(ANDROID_CONTROL_AF_TRIGGER).data.u8;
        if (trigger == ANDROID_CONTROL_AF_TRIGGER_START &&
            mState == ScanState::Idle &&
            afMode != ANDROID_CONTROL_AF_MODE_OFF) {
            startSweep(afMode);
        } else if (trigger == ANDROID_CONTROL_AF_TRIGGER_CANCEL) {
            cancelSweep();
        }
    }
}

void AutoFocusController::onFrameStart() {
    if (mState == ScanState::Idle) return;
    mDev->setFocusPosition(mSweepPos);
}

void AutoFocusController::onStats(const IpaStats &stats) {
    const float score = sumCentreFocus(stats.focusMetric);

    if (mState == ScanState::Idle) {
        if (mFramesSinceLastSweep < UINT32_MAX) mFramesSinceLastSweep++;

        /* Continuous-AF watches scene statistics for movement /
         * composition change. The snapshot-on-detect + count-while-
         * stable pattern is the natural motion gate: when the user
         * pans, every frame trips the ratio test, the snapshot is
         * replaced, and the counter resets to 1 — so the controller
         * never fires while panning is in progress. Once the camera
         * holds still, the ratio test starts passing, the counter
         * ticks up, and a fresh sweep launches once stability has
         * persisted for `mRetriggerDelay` frames. */
        if (mAfMode != ANDROID_CONTROL_AF_MODE_CONTINUOUS_PICTURE)
            return;

        float curRgb[3];
        sumCentreRgb(stats.rgbMean, curRgb);

        /* First idle frame after a sweep — capture snapshot and
         * skip change detection. mSceneFocusSnapshot==0 is the
         * sentinel commitSweep / reset / ctor leave behind. */
        if (mSceneFocusSnapshot <= 0.f) {
            mSceneFocusSnapshot = score;
            mSceneRgbSnapshot[0]    = curRgb[0];
            mSceneRgbSnapshot[1]    = curRgb[1];
            mSceneRgbSnapshot[2]    = curRgb[2];
            mSceneChangeCount       = 0;
            return;
        }

        const bool changed =
            outsideRatio(score,    mSceneFocusSnapshot, mRetriggerRatio) ||
            outsideRatio(curRgb[0], mSceneRgbSnapshot[0],    mRetriggerRatio) ||
            outsideRatio(curRgb[1], mSceneRgbSnapshot[1],    mRetriggerRatio) ||
            outsideRatio(curRgb[2], mSceneRgbSnapshot[2],    mRetriggerRatio);

        if (changed) {
            /* Refresh the snapshot to *current* values and (re)start
             * the countdown. Refresh is what gives the panning gate
             * its property — the snapshot moves with the scene, so a
             * sustained pan keeps tripping ratio and the counter
             * never advances past 1. */
            mSceneFocusSnapshot = score;
            mSceneRgbSnapshot[0]    = curRgb[0];
            mSceneRgbSnapshot[1]    = curRgb[1];
            mSceneRgbSnapshot[2]    = curRgb[2];
            mSceneChangeCount       = 1;
        } else if (mSceneChangeCount > 0) {
            mSceneChangeCount++;
        }

        /* Nuisance-scan rate limit. Even if the scene-change gate
         * fires legitimately, refuse to start another sweep within
         * `kMinFramesBetweenSweeps` of the last commit — keeps a
         * jittery scene from chain-triggering sweep-after-sweep
         * while AE / AWB keep readjusting and the controller never
         * lands on a stable focus. ~1 second at 30 fps. */
        constexpr uint32_t kMinFramesBetweenSweeps = 30u;
        const bool sweepCooldownDone =
            mFramesSinceLastSweep >= kMinFramesBetweenSweeps;

        /* Hold the retrigger off until AE has actually converged
         * on the new scene. Without this gate the AF re-fires while
         * the AE controller is still chasing — both stats inputs
         * (focus + RGB) are mid-transient, the snapshot used for
         * the next scene-change comparison gets contaminated, and
         * the sweep itself runs on an unstable image. The gate
         * keeps `mSceneChangeCount` at its current value, so once
         * AE settles the scan launches immediately rather than
         * having to wait `retriggerDelay` again. */
        const bool aeReady = !mIpa || mIpa->isAeConverged();
        if (mSceneChangeCount >= mRetriggerDelay && aeReady &&
            sweepCooldownDone) {
            ALOGD("AF: scene stabilised after change, starting sweep "
                  "(snap_sh=%.0f cur_sh=%.0f)",
                  (double)mSceneFocusSnapshot, (double)score);
            startSweep(mAfMode);
        }
        return;
    }

    /* After a VCM move, give the lens a few frames to settle before
     * reading the score. settleFrames was set by the previous step
     * based on its move magnitude. */
    if (mSettleFrames > 0) {
        mSettleFrames--;
        return;
    }

    ALOGD("AF: state=%d pos=%d score=%.0f best=%d/%.0f",
          (int)mState, mSweepPos, (double)score,
          mSweepBestPos, (double)mSweepBestScore);

    switch (mState) {
        case ScanState::Coarse1:
        case ScanState::Coarse2:
            advanceCoarse(score);
            break;
        case ScanState::Fine:
            advanceFine(score);
            break;
        case ScanState::Settle: {
            /* Two-condition focus gate (RPi libcamera pattern):
             *
             *   1. `score >= ratio × max`  — the Settle re-measure
             *      at the committed lens position still holds the
             *      peak the scan saw. Catches "the lens jumped
             *      somewhere noisy at the very end."
             *
             *   2. `min <= ratio × max`    — the scan actually
             *      *bracketed* a peak: at some point the score
             *      dipped at least `ratio` below the discrete max.
             *      Catches the flat-curve case where every sample
             *      is within a few percent of every other and
             *      "the highest one" is just sensor noise winning
             *      a coin flip. Without this gate the controller
             *      reports Focused on a curve that has no real
             *      peak — which is exactly the textureless / dark
             *      / occluded scene the algorithm should be
             *      humble about. */
            float scanMin = mSweepBestScore;
            for (const ScanSample &s : mScanData) {
                if (s.score < scanMin) scanMin = s.score;
            }
            const bool peakHolds =
                score    >= mContrastRatio * mSweepBestScore;
            const bool bracketed =
                scanMin  <= mContrastRatio * mSweepBestScore;
            const bool focused = peakHolds && bracketed;
            ALOGD("AF settle: score=%.0f best=%.0f min=%.0f "
                  "peakHolds=%d bracketed=%d → %s",
                  (double)score, (double)mSweepBestScore,
                  (double)scanMin,
                  peakHolds ? 1 : 0, bracketed ? 1 : 0,
                  focused ? "Focused" : "Failed");
            commitSweep(focused);
            return;
        }
        default:
            return;  /* Idle / Pdaf — unreachable here */
    }

    mDev->setFocusPosition(mSweepPos);
}

void AutoFocusController::reset() {
    if (mState != ScanState::Idle) {
        mIsp->setAwbLock(false);
        if (mIpa) mIpa->setAeLock(false);
    }
    mState              = ScanState::Idle;
    mAfMode             = ANDROID_CONTROL_AF_MODE_OFF;
    mSweepPos           = 0;
    mSweepStep          = 0;
    mSweepBestPos       = mVcmInfinity;
    mSweepBestScore     = 0.f;
    mSettleFrames       = 0;
    mScanData.clear();
    mCoarsePhaseStart   = 0;
    mFinePhaseStart     = 0;
    mCoarseReversed     = false;
    mSceneFocusSnapshot = 0.f;
    mSceneRgbSnapshot[0]    = 0.f;
    mSceneRgbSnapshot[1]    = 0.f;
    mSceneRgbSnapshot[2]    = 0.f;
    mSceneChangeCount   = 0;
    mFramesSinceLastSweep = UINT32_MAX;
    /* mFocusPosition kept — it reflects the physical VCM state,
     * not session state. */
}

AutoFocusController::Report AutoFocusController::report() const {
    Report r;
    r.afMode = mAfMode;
    r.afState = (mState != ScanState::Idle)
        ? ANDROID_CONTROL_AF_STATE_ACTIVE_SCAN
        : ANDROID_CONTROL_AF_STATE_FOCUSED_LOCKED;
    float diopter = (mFocusPosition - mVcmInfinity) / kVcmPerDiopter;
    if (diopter < 0)
        diopter = 0;
    r.focusDiopter = diopter;
    return r;
}

}; /* namespace android */
