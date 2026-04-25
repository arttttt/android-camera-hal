#ifndef HAL_3A_AUTO_FOCUS_CONTROLLER_H
#define HAL_3A_AUTO_FOCUS_CONTROLLER_H

#include <stdint.h>
#include <camera/CameraMetadata.h>

#include "ipa/IpaStats.h"

namespace android {

class V4l2Device;
class IspPipeline;
class SensorTuning;

/* Contrast-detect autofocus for a single-VCM sensor.
 *
 * Drives the V4L2 focus subdev through a coarse-to-fine sweep
 * around the current lens position, then validates the result
 * before reporting Focused / Failed. The ISP's AWB lock is held
 * across a sweep so brightness drift can't distort the score
 * curve. In CONTINUOUS_PICTURE the controller idles between
 * sweeps and re-triggers only when scene-statistics depart far
 * enough from the post-converge snapshot.
 *
 * State diagram:
 *
 *   Idle ──▶ [Pdaf]* ──▶ Coarse1 ──▶ Coarse2 ──▶ Fine ──▶ Settle ──▶ Idle
 *                            └──────────────────▶ Fine
 *
 *   *Pdaf reserved for future hybrid PDAF + CDAF on hardware that
 *    exposes phase data. None of the sensors we ship today have it
 *    (IMX179 / OV5693 / IMX219 are pre-PDAF Bayer parts), and there
 *    is no kernel-level phase output. The state value sits in the
 *    enum so a later refactor doesn't have to renumber everything;
 *    the runtime path skips straight to Coarse1. See
 *    raspberrypi/libcamera src/ipa/rpi/controller/rpi/af.cpp for
 *    the hybrid pattern when the hardware finally arrives. */
class AutoFocusController {
public:
    struct Report {
        uint8_t afState;       /* ANDROID_CONTROL_AF_STATE_*             */
        uint8_t afMode;        /* last observed ANDROID_CONTROL_AF_MODE  */
        float   focusDiopter;  /* 0 .. ~10, for ANDROID_LENS_FOCUS_DISTANCE */
    };

    /* `tuning` is optional — pass nullptr or a !isLoaded() instance and
     * the controller falls back to compile-time VCM defaults. When the
     * tuning provides AF data we consume it (inf/macro positions,
     * settle time, sweep step + scene-change knobs). */
    AutoFocusController(V4l2Device *dev, IspPipeline *isp,
                        const SensorTuning *tuning);

    /* Drive the state machine from request metadata. Reads AF_MODE,
     * LENS_FOCUS_DISTANCE, AF_TRIGGER. Continuous-mode re-triggering
     * is driven by scene statistics in onSharpnessStats, not by a
     * frame counter here. */
    void onSettings(const CameraMetadata &cm, uint32_t frameNumber);

    /* If a sweep is active, move the VCM to the current sweep position.
     * Called right after V4L2 dequeue, before CPU/GPU processing. */
    void onFrameStart();

    /* Advance the sweep against the per-patch sharpness grid for the
     * current frame, or evaluate scene-change in continuous-AF idle.
     * Caller hands over the same stats buffer the IPA has already
     * finished with. */
    void onSharpnessStats(
        const float sharpness[IpaStats::PATCH_Y][IpaStats::PATCH_X]);

    Report report() const;
    bool   isSweeping() const { return mState != ScanState::Idle; }

    /* Reset the state-machine to the initial post-construction
     * state. Used at camera close so a stale sweep doesn't leak
     * into the next session. Calibration (VCM positions, settle
     * frames) and the last focus position stay. */
    void reset();

private:
    enum class ScanState {
        Idle,      /* between sweeps; CAF watches for scene change       */
        Pdaf,      /* reserved for hybrid PDAF — see class comment       */
        Coarse1,   /* initial direction probe from current lens pos      */
        Coarse2,   /* reverse direction when Coarse1 didn't bracket peak */
        Fine,      /* +/- stepFine refinement around interpolated peak   */
        Settle,    /* validate final contrast, commit best position      */
    };

    /* One row of the score curve we accumulate during a sweep.
     * `parabolicPeak` reads three of these to interpolate a sub-step
     * peak position; `score` is the centre-ROI Tenengrad sum. */
    struct ScanSample {
        int32_t pos;
        float   score;
    };

    void   startSweep(uint8_t afMode);
    void   cancelSweep();
    void   beginCoarseFromCurrent();
    void   advanceCoarse(float score);
    void   advanceFine(float score);
    void   commitSweep(bool focused);
    void   recordCoarseSample(int32_t pos, float score);
    void   recordFineSample(int32_t pos, float score);
    int32_t parabolicPeak(const ScanSample s[3], int n) const;
    bool   nearLimit(int32_t pos, int32_t limit) const;

    V4l2Device  *mDev;
    IspPipeline *mIsp;

    /* Calibrated VCM positions — resolved from SensorTuning at
     * construction when available, otherwise compile-time defaults. */
    int32_t  mVcmInfinity;
    int32_t  mVcmMacroStart;
    int32_t  mVcmMacroEnd;
    int32_t  mVcmAutoEnd;
    int32_t  mVcmSettleFrames;

    /* HAL-side tunables resolved from SensorTuning::AfParams. */
    int32_t  mStepCoarse;
    int32_t  mStepFine;
    float    mContrastRatio;
    float    mRetriggerRatio;
    int32_t  mRetriggerDelay;
    bool     mPdafEnabled;

    uint8_t  mAfMode;
    int32_t  mFocusPosition;

    /* Sweep state — only meaningful when mState != Idle. */
    ScanState mState;
    int32_t   mSweepPos;      /* current / next VCM target           */
    int32_t   mSweepStep;     /* signed: positive toward macro       */
    int32_t   mSweepBestPos;
    float     mSweepBestScore;
    int32_t   mSettleFrames;
    /* Three-sample windows for parabolic interpolation, separate per
     * phase so the coarse-step and fine-step samples never share a
     * fit. Coarse keeps the running peak in slot [1] with the sample
     * before the rise in [0] and the first sample below the peak in
     * [2]. Fine fills slots in lens-travel order, then the closing
     * step puts the highest-score sample in [1] before invoking the
     * parabolic vertex calculation. */
    ScanSample mCoarseSamples[3];
    int        mCoarseSampleCount;
    ScanSample mFineSamples[3];
    int        mFineSampleCount;
    bool       mCoarseReversed;

    /* Continuous-AF scene-change tracking: snapshot of the per-frame
     * score we converged to, plus a frame counter that has to reach
     * `mRetriggerDelay` before a fresh sweep is launched. */
    float     mSceneScoreSnapshot;
    int32_t   mSceneChangeCount;
};

}; /* namespace android */

#endif /* HAL_3A_AUTO_FOCUS_CONTROLLER_H */
