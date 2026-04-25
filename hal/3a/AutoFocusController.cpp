#define LOG_TAG "Cam-AF"

#include "AutoFocusController.h"

#include <utils/Log.h>
#include <system/camera_metadata.h>

#include "V4l2Device.h"
#include "IspPipeline.h"
#include "sensor/SensorTuning.h"

namespace android {

namespace {

/* VCM positions are raw units exposed by the focuser subdev. The map to
 * diopters is linear within the usable range; 50 units per diopter.
 * Values outside [kVcmMin, kVcmMax] are rejected by the driver. The
 * inf / macro / auto-end / settle-frames values are module-specific
 * and come from SensorTuning (af.inf + af.inf_offset, etc.); these
 * constants are fallbacks for when tuning is unavailable. */
constexpr int32_t kVcmInfinityFallback  = 140;
constexpr int32_t kVcmMacroEndFallback  = 650;
constexpr int32_t kVcmAutoEndFallback   = 640;
constexpr int32_t kVcmSettleFallback    = 2;
constexpr int32_t kVcmSweepStep  = 25;
constexpr int32_t kVcmMin        = 0;
constexpr int32_t kVcmMax        = 1023;
constexpr float   kVcmPerDiopter = 50.0f;

constexpr uint32_t kContinuousRetriggerPeriod = 60; /* frames */
constexpr int32_t  kFramePeriodMsApprox       = 33; /* 30 fps preview */

/* Centre region of the patch grid: the inclusive [4, 11] band on each
 * axis covers the middle 50 % of the frame on both dimensions —
 * matches what the previous RGBA-domain sharpness measured on the
 * centre 1/4 of the rendered preview, so AF behaviour stays in the
 * same ballpark across the metric switch. */
constexpr int kRoiPatchLo = 4;
constexpr int kRoiPatchHi = 12;

} /* namespace */

AutoFocusController::AutoFocusController(V4l2Device *dev, IspPipeline *isp,
                                         const SensorTuning *tuning)
    : mDev(dev)
    , mIsp(isp)
    , mVcmInfinity(kVcmInfinityFallback)
    , mVcmMacroStart(0)         /* filled below */
    , mVcmMacroEnd(kVcmMacroEndFallback)
    , mVcmAutoEnd(kVcmAutoEndFallback)
    , mVcmSettleFrames(kVcmSettleFallback)
    , mAfMode(ANDROID_CONTROL_AF_MODE_OFF)
    , mFocusPosition(0)
    , mSweepActive(false)
    , mSweepPos(0)
    , mSweepStep(0)
    , mSweepEnd(0)
    , mSweepBestPos(0)
    , mSweepBestScore(0)
    , mSettleFrames(0)
{
    if (tuning && tuning->isLoaded() && tuning->hasAf()) {
        const SensorTuning::AfParams &af = tuning->af();
        if (af.moduleCalEnable) {
            /* `inf + inf_offset` is the calibrated real-world
             * infinity position (NVIDIA tuning convention — raw
             * `inf` is the mechanical limit, the offset trims it to
             * the lens's actual focus). Same for macro. */
            mVcmInfinity = af.infPos + af.infOffset;
            mVcmMacroEnd = af.macroPos + af.macroOffset;
        } else {
            /* Per-unit calibration not trusted (NVIDIA convention,
             * matches `lensShading.module_cal_enable=0`). Use the
             * mechanical anchors directly so a miscalibrated module
             * doesn't push the lens beyond its useful range. */
            mVcmInfinity = af.infPos;
            mVcmMacroEnd = af.macroPos;
        }
        mVcmAutoEnd = af.macroPos;
        /* Settle time as frames at 30 fps preview, rounded up, minimum 1. */
        int frames = (af.settleTimeMs + kFramePeriodMsApprox - 1) / kFramePeriodMsApprox;
        mVcmSettleFrames = frames > 0 ? frames : 1;
        ALOGD("AF tuning: cal=%d inf=%d macro=%d auto_end=%d settle=%d frame(s)",
              af.moduleCalEnable ? 1 : 0,
              mVcmInfinity, mVcmMacroEnd, mVcmAutoEnd, mVcmSettleFrames);
    }
    /* Macro mode starts at the midpoint between infinity and macro. */
    mVcmMacroStart = (mVcmInfinity + mVcmMacroEnd) / 2;

    mFocusPosition = mVcmInfinity;
    mSweepBestPos  = mVcmInfinity;
}

void AutoFocusController::startSweep(uint8_t afMode) {
    mSweepActive = true;
    mIsp->setAwbLock(true);
    mSettleFrames = 0;
    if (afMode == ANDROID_CONTROL_AF_MODE_MACRO) {
        mSweepPos = mVcmMacroStart;
        mSweepEnd = mVcmMacroEnd;
    } else {
        mSweepPos = mVcmInfinity;
        mSweepEnd = mVcmAutoEnd;
    }
    mSweepStep = kVcmSweepStep;
    mSweepBestPos = mSweepPos;
    mSweepBestScore = 0;
    ALOGD("AF sweep started (mode=%u, %d->%d step %d)",
          afMode, mSweepPos, mSweepEnd, mSweepStep);
}

void AutoFocusController::cancelSweep() {
    if (!mSweepActive)
        return;
    mSweepActive = false;
    mIsp->setAwbLock(false);
}

void AutoFocusController::onSettings(const CameraMetadata &cm,
                                     uint32_t frameNumber) {
    uint8_t afMode = ANDROID_CONTROL_AF_MODE_OFF;
    if (cm.exists(ANDROID_CONTROL_AF_MODE))
        afMode = *cm.find(ANDROID_CONTROL_AF_MODE).data.u8;

    /* Transition to OFF mid-sweep cleans up all state. */
    if (afMode == ANDROID_CONTROL_AF_MODE_OFF && mSweepActive)
        cancelSweep();
    mAfMode = afMode;

    /* Manual focus: LENS_FOCUS_DISTANCE (diopters = 1/m) -> VCM.
     *   0 diopters (infinity)  -> kVcmInfinity
     *   10 diopters (10 cm)    -> kVcmInfinity + 10 * kVcmPerDiopter */
    if (afMode == ANDROID_CONTROL_AF_MODE_OFF &&
        cm.exists(ANDROID_LENS_FOCUS_DISTANCE)) {
        float diopter = *cm.find(ANDROID_LENS_FOCUS_DISTANCE).data.f;
        int32_t pos = mVcmInfinity + (int32_t)(diopter * kVcmPerDiopter);
        if (pos < kVcmMin)
            pos = kVcmMin;
        if (pos > kVcmMax)
            pos = kVcmMax;
        mDev->setFocusPosition(pos);
        mFocusPosition = pos;
    }

    /* AF_TRIGGER_START kicks off a contrast-detect sweep. The trigger
     * is a no-op while AF_MODE=OFF — sweeps are only valid in AUTO /
     * MACRO / CONTINUOUS_PICTURE. */
    if (cm.exists(ANDROID_CONTROL_AF_TRIGGER)) {
        uint8_t trigger = *cm.find(ANDROID_CONTROL_AF_TRIGGER).data.u8;
        if (trigger == ANDROID_CONTROL_AF_TRIGGER_START &&
            !mSweepActive &&
            afMode != ANDROID_CONTROL_AF_MODE_OFF) {
            startSweep(afMode);
        } else if (trigger == ANDROID_CONTROL_AF_TRIGGER_CANCEL) {
            cancelSweep();
        }
    }

    /* Continuous AF: re-trigger every ~kContinuousRetriggerPeriod frames
     * while idle. */
    if (afMode == ANDROID_CONTROL_AF_MODE_CONTINUOUS_PICTURE &&
        !mSweepActive &&
        frameNumber % kContinuousRetriggerPeriod == 0) {
        startSweep(afMode);
        /* Prefer the previously converged position as a hint for the
         * reported state between measurements. */
        mSweepBestPos = mFocusPosition;
    }
}

void AutoFocusController::onFrameStart() {
    if (!mSweepActive)
        return;
    mDev->setFocusPosition(mSweepPos);
}

void AutoFocusController::onSharpnessStats(
    const float sharpness[IpaStats::PATCH_Y][IpaStats::PATCH_X]) {
    if (!mSweepActive || !sharpness)
        return;

    /* After a VCM move, the lens needs a couple of frames to settle
     * before the contrast reading is trustworthy. */
    if (mSettleFrames > 0) {
        mSettleFrames--;
        return;
    }

    /* Tenengrad sum over the centre 8x8 of the 16x16 patch grid.
     * AWB is locked across the sweep (mIsp->setAwbLock in
     * startSweep) so per-patch absolute scores stay comparable
     * within one sweep without per-frame normalisation. */
    float score = 0.f;
    for (int py = kRoiPatchLo; py < kRoiPatchHi; ++py) {
        for (int px = kRoiPatchLo; px < kRoiPatchHi; ++px) {
            score += sharpness[py][px];
        }
    }

    if (score > mSweepBestScore) {
        mSweepBestScore = score;
        mSweepBestPos = mSweepPos;
    }
    ALOGD("AF: pos=%d score=%.0f best=%d/%.0f",
          mSweepPos, (double)score,
          mSweepBestPos, (double)mSweepBestScore);

    mSweepPos += mSweepStep;
    if (mSweepPos > mSweepEnd) {
        mDev->setFocusPosition(mSweepBestPos);
        mFocusPosition = mSweepBestPos;
        mSweepActive = false;
        mIsp->setAwbLock(false);
        ALOGD("AF done: best=%d score=%.0f", mSweepBestPos,
              (double)mSweepBestScore);
        return;
    }

    mDev->setFocusPosition(mSweepPos);
    mSettleFrames = mVcmSettleFrames;
}

void AutoFocusController::reset() {
    if (mSweepActive) mIsp->setAwbLock(false);
    mSweepActive    = false;
    mAfMode         = ANDROID_CONTROL_AF_MODE_OFF;
    mSweepPos       = 0;
    mSweepStep      = 0;
    mSweepEnd       = 0;
    mSweepBestPos   = mVcmInfinity;
    mSweepBestScore = 0;
    mSettleFrames   = 0;
    /* mFocusPosition kept — it reflects the physical VCM state,
     * not session state. */
}

AutoFocusController::Report AutoFocusController::report() const {
    Report r;
    r.afMode = mAfMode;
    r.afState = mSweepActive
        ? ANDROID_CONTROL_AF_STATE_ACTIVE_SCAN
        : ANDROID_CONTROL_AF_STATE_FOCUSED_LOCKED;
    float diopter = (mFocusPosition - mVcmInfinity) / kVcmPerDiopter;
    if (diopter < 0)
        diopter = 0;
    r.focusDiopter = diopter;
    return r;
}

}; /* namespace android */
