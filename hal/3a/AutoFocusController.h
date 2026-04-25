#ifndef HAL_3A_AUTO_FOCUS_CONTROLLER_H
#define HAL_3A_AUTO_FOCUS_CONTROLLER_H

#include <stdint.h>
#include <vector>

#include <camera/CameraMetadata.h>

#include "ipa/IpaStats.h"

namespace android {

class V4l2Device;
class IspPipeline;
class SensorTuning;
class Ipa;

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
     * settle time, sweep step + scene-change knobs).
     *
     * `ipa` is the AE coordination channel. The controller asks it
     * whether AE has converged before launching a continuous-AF
     * retrigger (refusing to scan on a still-chasing exposure) and
     * tells it to hold the converged target across an active sweep
     * (so AE doesn't keep adjusting brightness mid-scan). May be
     * null for cold-bring-up — the AE checks then degenerate to
     * "always converged, never lock", matching StubIpa. */
    AutoFocusController(V4l2Device *dev, IspPipeline *isp, Ipa *ipa,
                        const SensorTuning *tuning);

    /* Drive the state machine from request metadata. Reads AF_MODE,
     * LENS_FOCUS_DISTANCE, AF_TRIGGER. Continuous-mode re-triggering
     * is driven by scene statistics in onSharpnessStats, not by a
     * frame counter here. */
    void onSettings(const CameraMetadata &cm, uint32_t frameNumber);

    /* If a sweep is active, move the VCM to the current sweep position.
     * Called right after V4L2 dequeue, before CPU/GPU processing. */
    void onFrameStart();

    /* Advance the sweep against the IPA stats grid for the current
     * frame, or evaluate scene-change in continuous-AF idle. Reads
     * both `sharpness` (Tenengrad) and `rgbMean` over the centre
     * 8x8 patches — the latter feeds the multi-channel scene-change
     * gate so the AWB / movement of the camera stops triggering
     * focus retriggers as soon as the sharpness signal alone wobbles.
     * Caller hands over the same stats buffer the IPA has already
     * finished with. */
    void onStats(const IpaStats &stats);

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
    /* AUTO / MACRO start a single-pass forward Coarse2 scan from
     * focusMin every time — full range, no direction-selection
     * mistakes possible. CONTINUOUS_PICTURE keeps the bidirectional
     * logic (Coarse1 from current with Coarse2 reversal as fallback)
     * because the cost of a full sweep on every retrigger would be
     * visually disruptive. RPi libcamera makes the same split. */
    void   beginCoarseFromCurrent(uint8_t afMode);
    void   advanceCoarse(float score);
    void   advanceFine(float score);
    void   commitSweep(bool focused);
    void   recordSample(int32_t pos, float score);
    /* Discrete argmax inside [start, end) over `mScanData`, plus a
     * parabolic refinement using the immediate neighbours when
     * argmax isn't on the boundary. Falls back to the discrete
     * sample at the boundary or when the parabolic fit degenerates. */
    int32_t findPeak(size_t start, size_t end) const;
    bool   nearLimit(int32_t pos, int32_t limit) const;
    void   setSettleForMove(int32_t fromPos, int32_t toPos);

    V4l2Device  *mDev;
    IspPipeline *mIsp;
    Ipa         *mIpa;

    /* Calibrated VCM positions — resolved from SensorTuning at
     * construction when available, otherwise compile-time defaults. */
    int32_t  mVcmInfinity;
    int32_t  mVcmMacroStart;
    int32_t  mVcmMacroEnd;
    int32_t  mVcmAutoEnd;

    /* HAL-side tunables resolved from SensorTuning::AfParams. */
    int32_t  mStepCoarse;
    int32_t  mStepFine;
    float    mContrastRatio;
    float    mRetriggerRatio;
    int32_t  mRetriggerDelay;
    bool     mPdafEnabled;
    int32_t  mSettleFramesCoarse;
    int32_t  mSettleFramesFine;

    uint8_t  mAfMode;
    int32_t  mFocusPosition;

    /* Sweep state — only meaningful when mState != Idle. */
    ScanState mState;
    int32_t   mSweepPos;      /* current / next VCM target           */
    int32_t   mSweepStep;     /* signed: positive toward macro       */
    int32_t   mSweepBestPos;
    float     mSweepBestScore;
    int32_t   mSettleFrames;
    /* Full per-sweep score history. Every Coarse / Fine sample
     * appends; cleared on startSweep. Parabolic peak interpolation
     * runs over phase-bounded slices of this vector so coarse-step
     * and fine-step samples don't get mixed in one fit (different
     * sample spacings would skew the vertex calculation). The full
     * history is what RPi libcamera's `scanData_` does — keeping
     * every reading lets the closing peak search look at the
     * complete curve rather than a sliding 3-sample window that
     * loses points on slow descents. */
    std::vector<ScanSample> mScanData;
    /* Index in `mScanData` where Coarse-phase samples begin. Coarse2
     * after a reversal-with-reset advances this past the Coarse1
     * remnants so the Coarse → Fine handoff sees only the
     * meaningful direction. */
    size_t mCoarsePhaseStart;
    /* Index where Fine-phase samples begin. -1 / vector-size while
     * Coarse is still running. */
    size_t mFinePhaseStart;
    bool   mCoarseReversed;

    /* Continuous-AF scene-change tracking. Snapshot is multi-channel
     * — sharpness plus per-channel RGB mean over the centre patches —
     * because sharpness alone is a poor signal for scene movement
     * (focus drift moves it without the scene actually changing).
     *
     * The counter semantics follow RPi libcamera: when any channel
     * deviates from the snapshot beyond `retriggerRatio`, the
     * snapshot is *replaced with the current values* and the counter
     * resets to 1. While the scene is stable the counter ticks up;
     * a sweep fires once it reaches `retriggerDelay`. The
     * snapshot-on-detect refresh is the motion gate: panning the
     * camera keeps replacing the snapshot every frame, so the
     * counter never tops out and the controller stays idle until
     * the user holds the camera still long enough for the
     * retrigger-delay to elapse. The counter increments only after
     * the first detect, so on a fully stable boot we don't sweep
     * spuriously. */
    float     mSceneFocusSnapshot;
    float     mSceneRgbSnapshot[3];
    int32_t   mSceneChangeCount;
    /* Frames since the last sweep committed. Used as a nuisance-
     * scan rate limiter — even if the scene-change gate fires
     * legitimately, we don't run another sweep within
     * `kMinFramesBetweenSweeps` of the previous one. Stops the
     * controller from cycle-locking on a moving / jittering scene
     * where each retrigger triggers the next. */
    uint32_t  mFramesSinceLastSweep;
};

}; /* namespace android */

#endif /* HAL_3A_AUTO_FOCUS_CONTROLLER_H */
