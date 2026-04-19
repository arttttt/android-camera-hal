#define LOG_TAG "Cam-AF"

#include "AutoFocusController.h"

#include <utils/Log.h>
#include <system/camera_metadata.h>

#include "V4l2Device.h"
#include "IspPipeline.h"

namespace android {

namespace {

/* VCM positions are raw units exposed by the focuser subdev. The map to
 * diopters is linear within the usable range: 140 is infinity, every
 * 50 units is one diopter. Values outside [kVcmMin, kVcmMax] are
 * rejected by the driver. */
constexpr int32_t kVcmInfinity   = 140;
constexpr int32_t kVcmMacroStart = 400;
constexpr int32_t kVcmMacroEnd   = 650;
constexpr int32_t kVcmAutoEnd    = 640;
constexpr int32_t kVcmSweepStep  = 25;
constexpr int32_t kVcmMin        = 0;
constexpr int32_t kVcmMax        = 1023;
constexpr float   kVcmPerDiopter = 50.0f;

constexpr uint32_t kContinuousRetriggerPeriod = 60; /* frames */
constexpr int32_t  kSettleFrames = 2;

/* Normalised Laplacian on the centre 1/4 of the frame. Dividing by the
 * mean brightness keeps the score comparable across sweep steps even
 * when AE/AWB shifts brightness slightly. */
uint64_t measureSharpness(const uint8_t *rgba, unsigned w, unsigned h) {
    unsigned cx = w / 4;
    unsigned cy = h / 4;
    unsigned cw = w / 2;
    unsigned ch = h / 2;
    uint64_t gradSum = 0;
    uint64_t brightSum = 0;
    unsigned nSamples = 0;
    for (unsigned y = cy + 1; y < cy + ch - 1; y += 4) {
        const uint8_t *row  = rgba + y * w * 4;
        const uint8_t *rowU = rgba + (y - 1) * w * 4;
        const uint8_t *rowD = rgba + (y + 1) * w * 4;
        for (unsigned x = cx + 1; x < cx + cw - 1; x += 4) {
            int g  = row [x * 4 + 1];
            int gl = row [(x - 1) * 4 + 1];
            int gr = row [(x + 1) * 4 + 1];
            int gu = rowU[x * 4 + 1];
            int gd = rowD[x * 4 + 1];
            int lapH = 2 * g - gl - gr;
            int lapV = 2 * g - gu - gd;
            gradSum += (lapH < 0 ? -lapH : lapH) + (lapV < 0 ? -lapV : lapV);
            brightSum += g;
            nSamples++;
        }
    }
    if (brightSum == 0 || nSamples == 0)
        return 0;
    return gradSum * 1000 / (brightSum / nSamples);
}

} /* namespace */

AutoFocusController::AutoFocusController(V4l2Device *dev, IspPipeline *isp)
    : mDev(dev)
    , mIsp(isp)
    , mAfMode(ANDROID_CONTROL_AF_MODE_OFF)
    , mFocusPosition(kVcmInfinity)
    , mSweepActive(false)
    , mSweepPos(0)
    , mSweepStep(0)
    , mSweepEnd(0)
    , mSweepBestPos(kVcmInfinity)
    , mSweepBestScore(0)
    , mSettleFrames(0) {
}

void AutoFocusController::startSweep(uint8_t afMode) {
    mSweepActive = true;
    mIsp->setAwbLock(true);
    mSettleFrames = 0;
    if (afMode == ANDROID_CONTROL_AF_MODE_MACRO) {
        mSweepPos = kVcmMacroStart;
        mSweepEnd = kVcmMacroEnd;
    } else {
        mSweepPos = kVcmInfinity;
        mSweepEnd = kVcmAutoEnd;
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
        int32_t pos = kVcmInfinity + (int32_t)(diopter * kVcmPerDiopter);
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

void AutoFocusController::onFrameData(const uint8_t *rgba,
                                      unsigned width, unsigned height) {
    if (!mSweepActive || !rgba)
        return;

    /* After a VCM move, the lens needs a couple of frames to settle
     * before the contrast reading is trustworthy. */
    if (mSettleFrames > 0) {
        mSettleFrames--;
        return;
    }

    uint64_t score = measureSharpness(rgba, width, height);
    if (score > mSweepBestScore) {
        mSweepBestScore = score;
        mSweepBestPos = mSweepPos;
    }
    ALOGD("AF: pos=%d score=%llu best=%d/%llu",
          mSweepPos, (unsigned long long)score,
          mSweepBestPos, (unsigned long long)mSweepBestScore);

    mSweepPos += mSweepStep;
    if (mSweepPos > mSweepEnd) {
        mDev->setFocusPosition(mSweepBestPos);
        mFocusPosition = mSweepBestPos;
        mSweepActive = false;
        mIsp->setAwbLock(false);
        ALOGD("AF done: best=%d score=%llu", mSweepBestPos,
              (unsigned long long)mSweepBestScore);
        return;
    }

    mDev->setFocusPosition(mSweepPos);
    mSettleFrames = kSettleFrames;
}

AutoFocusController::Report AutoFocusController::report() const {
    Report r;
    r.afMode = mAfMode;
    r.afState = mSweepActive
        ? ANDROID_CONTROL_AF_STATE_ACTIVE_SCAN
        : ANDROID_CONTROL_AF_STATE_FOCUSED_LOCKED;
    float diopter = (mFocusPosition - kVcmInfinity) / kVcmPerDiopter;
    if (diopter < 0)
        diopter = 0;
    r.focusDiopter = diopter;
    return r;
}

}; /* namespace android */
