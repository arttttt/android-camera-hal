#define LOG_TAG "Cam-AF"

#include "AutoFocusController.h"

#include <math.h>

#include <utils/Log.h>
#include <system/camera_metadata.h>

#include "V4l2Device.h"
#include "IspPipeline.h"
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
constexpr int32_t kVcmSettleFallback   = 2;
constexpr int32_t kVcmMin              = 0;
constexpr int32_t kVcmMax              = 1023;
constexpr float   kVcmPerDiopter       = 50.0f;

constexpr int32_t kFramePeriodMsApprox = 33;  /* 30 fps preview */

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

float sumCentreSharpness(
    const float sharpness[IpaStats::PATCH_Y][IpaStats::PATCH_X]) {
    float s = 0.f;
    for (int py = kRoiPatchLo; py < kRoiPatchHi; ++py) {
        for (int px = kRoiPatchLo; px < kRoiPatchHi; ++px) {
            s += sharpness[py][px];
        }
    }
    return s;
}

int32_t clampVcm(int32_t pos) {
    if (pos < kVcmMin) return kVcmMin;
    if (pos > kVcmMax) return kVcmMax;
    return pos;
}

} /* namespace */

AutoFocusController::AutoFocusController(V4l2Device *dev, IspPipeline *isp,
                                         const SensorTuning *tuning)
    : mDev(dev)
    , mIsp(isp)
    , mVcmInfinity(kVcmInfinityFallback)
    , mVcmMacroStart(0)
    , mVcmMacroEnd(kVcmMacroEndFallback)
    , mVcmAutoEnd(kVcmAutoEndFallback)
    , mVcmSettleFrames(kVcmSettleFallback)
    , mStepCoarse(kDefStepCoarse)
    , mStepFine(kDefStepFine)
    , mContrastRatio(kDefContrastRatio)
    , mRetriggerRatio(kDefRetriggerRatio)
    , mRetriggerDelay(kDefRetriggerDelay)
    , mPdafEnabled(false)
    , mAfMode(ANDROID_CONTROL_AF_MODE_OFF)
    , mFocusPosition(0)
    , mState(ScanState::Idle)
    , mSweepPos(0)
    , mSweepStep(0)
    , mSweepBestPos(0)
    , mSweepBestScore(0.f)
    , mSettleFrames(0)
    , mCoarseSampleCount(0)
    , mLastSeen{0, 0.f}
    , mFineSampleCount(0)
    , mCoarseReversed(false)
    , mSceneScoreSnapshot(0.f)
    , mSceneChangeCount(0)
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
        int frames = (af.settleTimeMs + kFramePeriodMsApprox - 1) / kFramePeriodMsApprox;
        mVcmSettleFrames = frames > 0 ? frames : 1;

        if (af.stepCoarse     > 0)   mStepCoarse     = af.stepCoarse;
        if (af.stepFine       > 0)   mStepFine       = af.stepFine;
        if (af.contrastRatio  > 0.f) mContrastRatio  = af.contrastRatio;
        if (af.retriggerRatio > 0.f) mRetriggerRatio = af.retriggerRatio;
        if (af.retriggerDelay > 0)   mRetriggerDelay = af.retriggerDelay;
        mPdafEnabled = af.pdafEnabled;

        ALOGD("AF tuning: cal=%d inf=%d macro=%d auto_end=%d settle=%d frame(s) "
              "step=%d/%d ratios=%.2f/%.2f delay=%d pdaf=%d",
              af.moduleCalEnable ? 1 : 0,
              mVcmInfinity, mVcmMacroEnd, mVcmAutoEnd, mVcmSettleFrames,
              mStepCoarse, mStepFine,
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

void AutoFocusController::beginCoarseFromCurrent() {
    /* Decide where to start the sweep. RPi pattern: anchor at the
     * current lens position when there's room on both sides; clamp
     * to the nearer limit otherwise. */
    if (nearLimit(mFocusPosition, mVcmInfinity)) {
        mSweepPos  = mVcmInfinity;
        mSweepStep = mStepCoarse;
        mState     = ScanState::Coarse2;       /* forward only */
        mCoarseReversed = true;                 /* no reversal allowed */
    } else if (nearLimit(mFocusPosition, mVcmAutoEnd)) {
        mSweepPos  = mVcmAutoEnd;
        mSweepStep = -mStepCoarse;
        mState     = ScanState::Coarse2;       /* backward only */
        mCoarseReversed = true;
    } else {
        mSweepPos  = mFocusPosition;
        mSweepStep = -mStepCoarse;             /* probe toward infinity */
        mState     = ScanState::Coarse1;
        mCoarseReversed = false;
    }
    mSweepPos = clampVcm(mSweepPos);
}

void AutoFocusController::startSweep(uint8_t afMode) {
    mIsp->setAwbLock(true);
    /* Lock AE for the duration of the sweep — without it, the IPA's
     * exposure / gain controller crawls toward its setpoint while
     * the lens is moving and the resulting brightness drift
     * skews the sharpness scores Coarse → Fine → Settle compare
     * against each other. The lock releases on commitSweep /
     * cancelSweep / reset. */
    mIsp->setAeLock(true);
    mSettleFrames      = 0;
    mSweepBestScore    = 0.f;
    mCoarseSampleCount = 0;
    mLastSeen          = {0, 0.f};
    mFineSampleCount   = 0;
    mSceneChangeCount  = 0;

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
        beginCoarseFromCurrent();
        mSweepBestPos   = mSweepPos;
    }

    ALOGD("AF sweep started (mode=%u, start=%d step=%d state=%s)",
          afMode, mSweepPos, mSweepStep,
          mState == ScanState::Coarse1 ? "Coarse1" : "Coarse2");
}

void AutoFocusController::cancelSweep() {
    if (mState == ScanState::Idle) return;
    mIsp->setAwbLock(false);
    mIsp->setAeLock(false);
    mState = ScanState::Idle;
}

void AutoFocusController::recordCoarseSample(int32_t pos, float score) {
    /* Track running max + maintain a 3-sample window so
     * parabolicPeak can interpolate sub-step accuracy off coarse
     * data. Slot [1] is always the running peak, [0] the immediate
     * predecessor sample (regardless of whether it was itself a
     * peak), [2] the first sample below the running peak. The
     * predecessor matters: feeding the parabolic vertex math with a
     * non-adjacent "old peak" sample skews the fit toward whichever
     * side has the more distant neighbour and lands the interpolated
     * peak well off the actual maximum. */
    if (score > mSweepBestScore) {
        if (mCoarseSampleCount > 0) {
            mCoarseSamples[0] = mLastSeen;
        }
        mCoarseSamples[1].pos   = pos;
        mCoarseSamples[1].score = score;
        mCoarseSampleCount = mCoarseSampleCount < 2 ? mCoarseSampleCount + 1 : 2;
        mSweepBestScore = score;
        mSweepBestPos   = pos;
    } else if (mCoarseSampleCount == 2) {
        mCoarseSamples[2].pos   = pos;
        mCoarseSamples[2].score = score;
        mCoarseSampleCount = 3;
    }
    mLastSeen.pos   = pos;
    mLastSeen.score = score;
}

void AutoFocusController::recordFineSample(int32_t pos, float score) {
    /* Fine fills slots in lens-travel order; the closing step in
     * advanceFine sorts the array so the highest-score sample lands
     * in slot [1] before the parabolic vertex calculation. */
    if (mFineSampleCount < 3) {
        mFineSamples[mFineSampleCount].pos   = pos;
        mFineSamples[mFineSampleCount].score = score;
        mFineSampleCount++;
    }
    if (score > mSweepBestScore) {
        mSweepBestScore = score;
        mSweepBestPos   = pos;
    }
}

int32_t AutoFocusController::parabolicPeak(const ScanSample s[3], int n) const {
    if (n < 3) return mSweepBestPos;
    const float x0 = (float)s[0].pos, y0 = s[0].score;
    const float x1 = (float)s[1].pos, y1 = s[1].score;
    const float x2 = (float)s[2].pos, y2 = s[2].score;
    /* Parabolic vertex: x* = x1 - 0.5 * ((x1-x0)^2(y1-y2) - (x1-x2)^2(y1-y0))
     *                            / ((x1-x0)(y1-y2) - (x1-x2)(y1-y0))
     * Numerically stable form factoring out common diffs. Caller
     * places the highest-score sample at index 1. */
    const float a = (x1 - x0);
    const float b = (x1 - x2);
    const float p = a * (y1 - y2);
    const float q = b * (y1 - y0);
    const float denom = p - q;
    if (fabsf(denom) < 1e-6f) return mSweepBestPos;
    const float vx = x1 - 0.5f * (a * p - b * q) / denom;
    int32_t out = (int32_t)(vx + 0.5f);
    /* Sanity-clamp to between the outer samples — a degenerate fit
     * can extrapolate well outside the bracket. */
    int32_t lo = s[0].pos < s[2].pos ? s[0].pos : s[2].pos;
    int32_t hi = s[0].pos < s[2].pos ? s[2].pos : s[0].pos;
    if (out < lo) out = lo;
    if (out > hi) out = hi;
    return clampVcm(out);
}

void AutoFocusController::advanceCoarse(float score) {
    recordCoarseSample(mSweepPos, score);

    const bool peakPassed =
        mSweepBestScore > 0.f &&
        score < mContrastRatio * mSweepBestScore;
    const bool atLimit =
        (mSweepStep > 0 && mSweepPos >= mVcmAutoEnd) ||
        (mSweepStep < 0 && mSweepPos <= mVcmInfinity);

    /* Coarse1 may discover its first sample is already low — in that
     * case the peak isn't on this side of the start position; reverse
     * direction and try the other side. */
    if (mState == ScanState::Coarse1 && !mCoarseReversed &&
        mCoarseSampleCount == 1 &&
        score < mContrastRatio * mSweepBestScore) {
        mSweepStep      = -mSweepStep;
        mState          = ScanState::Coarse2;
        mCoarseReversed = true;
        /* Restart from the original position so Coarse2 covers the
         * other half of the range cleanly. */
        mSweepPos       = mSweepBestPos;
    }

    if (peakPassed || atLimit) {
        /* Sub-step coarse peak from the 3-sample parabolic fit if
         * we have all three; falls back to mSweepBestPos otherwise.
         * Using this as Fine's centre ensures the ±stepFine bracket
         * brackets the real peak even when it sits between coarse
         * samples. */
        const int32_t coarsePk =
            parabolicPeak(mCoarseSamples, mCoarseSampleCount);
        /* Step into Fine, ±stepFine away from the interpolated peak,
         * direction continuing where Coarse left off so the lens
         * keeps moving the same way. */
        if (mSweepStep > 0) {
            mSweepPos = clampVcm(coarsePk - mStepFine);
            mSweepStep = mStepFine;
        } else {
            mSweepPos = clampVcm(coarsePk + mStepFine);
            mSweepStep = -mStepFine;
        }
        mState           = ScanState::Fine;
        mFineSampleCount = 0;
        return;
    }

    mSweepPos = clampVcm(mSweepPos + mSweepStep);
}

void AutoFocusController::advanceFine(float score) {
    recordFineSample(mSweepPos, score);

    /* Three Fine samples is enough to confirm the parabolic peak.
     * Past that we'd just be fighting VCM noise. */
    if (mFineSampleCount >= 3) {
        /* Numerical stability for the parabolic vertex needs the
         * highest-score sample anchored at slot [1]. Fine collected
         * the trio in lens-travel order, so swap if needed. */
        int maxIdx = 0;
        if (mFineSamples[1].score > mFineSamples[maxIdx].score) maxIdx = 1;
        if (mFineSamples[2].score > mFineSamples[maxIdx].score) maxIdx = 2;
        if (maxIdx != 1) {
            ScanSample tmp = mFineSamples[1];
            mFineSamples[1] = mFineSamples[maxIdx];
            mFineSamples[maxIdx] = tmp;
        }
        const int32_t finePk = parabolicPeak(mFineSamples, 3);
        mSweepPos = finePk;
        mState    = ScanState::Settle;
        return;
    }
    mSweepPos = clampVcm(mSweepPos + mSweepStep);
}

void AutoFocusController::commitSweep(bool focused) {
    /* Drive to the Fine-interpolated peak (mSweepPos at this point —
     * set by advanceFine to the parabolic vertex). Falls back to
     * mSweepBestPos only if Fine never managed a fit, which happens
     * when Coarse hit a range limit before bracketing the peak. */
    const int32_t finalPos = (mFineSampleCount >= 3)
                             ? mSweepPos
                             : mSweepBestPos;
    mDev->setFocusPosition(finalPos);
    mFocusPosition = finalPos;
    mIsp->setAwbLock(false);
    mIsp->setAeLock(false);
    mSceneScoreSnapshot = mSweepBestScore;
    mSceneChangeCount   = 0;
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

void AutoFocusController::onSharpnessStats(
    const float sharpness[IpaStats::PATCH_Y][IpaStats::PATCH_X]) {
    if (!sharpness) return;
    const float score = sumCentreSharpness(sharpness);

    if (mState == ScanState::Idle) {
        /* CONTINUOUS_PICTURE: watch the sharpness signal and re-trigger
         * a sweep when it's drifted far enough from where we converged.
         * The symmetric ratio test catches both "scene got blurrier"
         * and "scene got crisper than what we locked on" — both mean
         * the lens needs another look. */
        if (mAfMode == ANDROID_CONTROL_AF_MODE_CONTINUOUS_PICTURE &&
            mSceneScoreSnapshot > 0.f) {
            const bool changed =
                score < mRetriggerRatio * mSceneScoreSnapshot ||
                mSceneScoreSnapshot < mRetriggerRatio * score;
            if (changed) {
                mSceneChangeCount++;
                if (mSceneChangeCount >= mRetriggerDelay) {
                    ALOGD("AF: scene change detected (snap=%.0f cur=%.0f), "
                          "starting sweep",
                          (double)mSceneScoreSnapshot, (double)score);
                    startSweep(mAfMode);
                }
            } else {
                mSceneChangeCount = 0;
            }
        }
        return;
    }

    /* After a VCM move, give the lens a couple of frames to settle
     * before reading the score. */
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
            /* The Settle frame measures sharpness at the committed
             * peak position. If it holds the contrastRatio threshold
             * relative to the running best, declare focus achieved;
             * otherwise the curve was too flat / too noisy to trust. */
            const bool focused =
                score >= mContrastRatio * mSweepBestScore;
            commitSweep(focused);
            return;
        }
        default:
            return;  /* Idle / Pdaf — unreachable here */
    }

    mDev->setFocusPosition(mSweepPos);
    mSettleFrames = mVcmSettleFrames;
}

void AutoFocusController::reset() {
    if (mState != ScanState::Idle) {
        mIsp->setAwbLock(false);
        mIsp->setAeLock(false);
    }
    mState              = ScanState::Idle;
    mAfMode             = ANDROID_CONTROL_AF_MODE_OFF;
    mSweepPos           = 0;
    mSweepStep          = 0;
    mSweepBestPos       = mVcmInfinity;
    mSweepBestScore     = 0.f;
    mSettleFrames       = 0;
    mCoarseSampleCount  = 0;
    mLastSeen           = {0, 0.f};
    mFineSampleCount    = 0;
    mCoarseReversed     = false;
    mSceneScoreSnapshot = 0.f;
    mSceneChangeCount   = 0;
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
