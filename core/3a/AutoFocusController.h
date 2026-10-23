#ifndef HAL_3A_AUTO_FOCUS_CONTROLLER_H
#define HAL_3A_AUTO_FOCUS_CONTROLLER_H

#include <stdint.h>
#include <mutex>
#include <vector>

#include <utils/Timers.h>

#include "mochacamera/Controls.h"
#include "mochacamera/FrameSettings.h"

#include "AfResult.h"
#include "ipa/IpaStats.h"

namespace android {

class FrameMap;
class SensorTuning;

/* Contrast-detect autofocus for a single-VCM sensor.
 *
 * Drives the lens through a coarse-to-fine sweep around its current
 * position, then validates the result before reporting Focused /
 * Failed. It does not touch the lens itself: process() returns the
 * position to move to, and the coordinator writes it.
 *
 * Threading: the sweep's state belongs to the thread that calls
 * process() (the finish engine) and changes nowhere else. onSettings
 * runs on the capture engine and only queues what the request asks for;
 * process() applies the queue at the start of its tick. report()
 * hands out the state process() last published, so the result
 * thread never reads fields another thread is writing. The ISP's AWB lock is held
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
        mocha::camera::AfState afState;
        mocha::camera::AfMode  afMode;        /* last observed */
        float   focusDiopter;  /* 0 .. ~10, the lens focus distance */
    };

    /* AF region of interest in IpaStats patch grid coordinates. */
    using FocusRoi = PatchRoi;

    /* `tuning` is optional — pass nullptr or a !isLoaded() instance and
     * the controller falls back to compile-time VCM defaults. When the
     * tuning provides AF data we consume it (inf/macro positions,
     * settle time, sweep step + scene-change knobs).
     *
     * No ISP / Ipa pointers any more. AE-converged signal
     * comes in as an argument to `process`; AE / AWB locking across a
     * sweep is published to the coordinator via `AfResult::sweeping`
     * — the coordinator decides which back-ends (Ipa::setAeLock,
     * the AWB hold in Ipa3A) to nudge. */
    /* `frameMap` maps the active array AF_REGIONS come in onto the
     * running mode's frame; it is changed only while no settings come
     * (between configurations). */
    AutoFocusController(const SensorTuning *tuning, const FrameMap *frameMap);

    /* Queue what the request asks of AF: AF_MODE, LENS_FOCUS_DISTANCE,
     * AF_TRIGGER. Applied in request order at the start of the next
     * process() tick. The AF region is parsed here and published at
     * once (see currentFocusRoi). Continuous-mode re-triggering is
     * driven by scene statistics in process(), not from here. */
    void onSettings(const mocha::camera::FrameSettings &settings, uint32_t frameNumber);

    /* Advance the sweep against the IPA stats grid for the current
     * frame, or evaluate scene-change in continuous-AF idle. Reads
     * both `sharpness` (Tenengrad) and `rgbMean` over the centre
     * 8x8 patches — the latter feeds the multi-channel scene-change
     * gate so the AWB / movement of the camera stops triggering
     * focus retriggers as soon as the sharpness signal alone wobbles.
     * Caller hands over the same stats buffer the IPA has already
     * finished with.
     *
     * `fresh` says these stats were not handed in on an earlier tick;
     * `statsExposureStartNs` is when the frame they come from began
     * exposing (systemTime, 0 = unknown). A sweep only samples fresh
     * stats of a frame exposed after the lens settled.
     *
     * `aeConverged` gates continuous-AF retrigger — refuses to scan
     * while AE is still chasing exposure (brightness changes from AE
     * catching up would look like scene changes). The coordinator
     * passes the AE controller's current convergence flag.
     *
     * Returned AfResult.sweeping tells the coordinator to hold AE / AWB
     * locked for as long as it is true. `vcmPosition`, when
     * set, is where the lens has to go now; the coordinator writes it
     * on this same tick, which keeps a sweep step on the frame the
     * state machine decided it. */
    AfResult process(const IpaStats &stats, bool fresh,
                     int64_t statsExposureStartNs, bool aeConverged);

    /* The state as of the last process() tick. Safe from any thread. */
    Report report() const;

    /* Snapshot the AF region the stats engine should restrict its
     * Sobel / greenSq accumulation to. Updated from the framework's
     * the request's AF region in onSettings; defaults to the
     * compile-time FOCUS_ROI_* centre rectangle when no app sets a
     * region (or when the region is the default full-frame one).
     * Thread-safe — onSettings runs on the capture engine, as does
     * this getter when it hands a frame to the stats engine; the
     * finish engine reads it for AE / AWB metering. */
    FocusRoi currentFocusRoi() const;

    /* The patches a frame's AF region covers on the running mode's
     * frame, grown to the tuning's minimum; the centre default without
     * a region that takes part (regionPatches). */
    FocusRoi roiFor(const mocha::camera::FrameSettings &settings) const;

    /* Reset the state-machine to the initial post-construction
     * state. Used at camera close, with the threads stopped, so a
     * stale sweep doesn't leak into the next session. Calibration
     * (VCM positions, settle frames) and the last focus position
     * stay. */
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

    /* A passive sweep is CONTINUOUS_PICTURE refocusing on its own on
     * a scene change (PASSIVE_SCAN); a triggered one answers
     * AF_TRIGGER_START (ACTIVE_SCAN) and ends locked. */
    enum class SweepKind { Passive, Triggered };

    void   startSweep(mocha::camera::AfMode afMode, SweepKind kind);
    void   cancelSweep();
    /* CONTINUOUS_PICTURE lock: report the last sweep's verdict as
     * FOCUSED_LOCKED / NOT_FOCUSED_LOCKED and stop refocusing until
     * CANCEL or a mode change. */
    void   lockOnVerdict();
    /* Highest VCM position a sweep in the current mode may reach:
     * the calibrated macro end for MACRO, macroPos capped by it
     * otherwise. Every range check uses this, so a scan always ends
     * at a position clampVcm lets it reach. */
    int32_t sweepUpperLimit() const;
    /* AUTO / MACRO start a single-pass forward Coarse2 scan from
     * focusMin every time — full range, no direction-selection
     * mistakes possible. CONTINUOUS_PICTURE keeps the bidirectional
     * logic (Coarse1 from current with Coarse2 reversal as fallback)
     * because the cost of a full sweep on every retrigger would be
     * visually disruptive. RPi libcamera makes the same split. */
    void   beginCoarseFromCurrent(mocha::camera::AfMode afMode);
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

    /* Clamp `pos` to the calibrated VCM range
     * [mVcmInfinity, mVcmMacroEnd] — sweep / manual-focus / etc.
     * never push the lens past the points the tuning calibrated, so
     * a slider beyond MIN_FOCUS_DISTANCE just rests at the macro
     * extreme instead of driving the actuator into the hardware
     * stop where it stops responding. */
    int32_t clampVcm(int32_t pos) const;

    /* What one request asked of AF, queued by onSettings. */
    struct Command {
        mocha::camera::AfMode afMode;
        bool    hasFocusDistance;  /* manual focus: AF_MODE_OFF + LENS_FOCUS_DISTANCE */
        float   focusDistance;     /* diopters */
        mocha::camera::AfTrigger trigger;
    };
    void applyCommand(const Command &cmd);

    /* Ask for the lens to move to `pos` on this tick. */
    void moveLens(int32_t pos) {
        mLensTarget  = pos;
        mLensMovedNs = systemTime();
    }
    void publishReport();
    Report buildReport() const;

    const FrameMap *mFrameMap;

    /* onSettings → process(): requests not yet applied, in order.
     * Consecutive requests without a trigger collapse into the last
     * one, so the queue stays short however far the pipeline lags. */
    std::mutex           mCommandMutex;
    std::vector<Command> mCommands;

    /* Lens move decided on the current tick; harvested into AfResult. */
    Optional<int32_t>    mLensTarget;

    /* process() → report(). */
    mutable std::mutex   mReportMutex;
    Report               mPublished;

    /* Camera2 AF state machine, set on
     * every transition: mode change, trigger, sweep start, commit. */
    mocha::camera::AfState mAfState;
    SweepKind    mSweepKind;
    /* CONTINUOUS_PICTURE: locked by a trigger; no passive sweeps. */
    bool         mCafLocked;
    /* CONTINUOUS_PICTURE: START arrived during a passive sweep; lock
     * when it ends. */
    bool         mLockPending;
    /* Verdict of the last sweep, for locking without a new one. */
    bool         mHaveVerdict;
    bool         mLastFocused;
    /* AF region the last sweep focused on; a trigger with another one
     * refocuses before locking. */
    FocusRoi     mLastSweepRoi;

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
    int64_t  mSettleNs;      /* VCM settle after a move (tuning settle_time) */
    uint32_t mMinFramesBetweenSweeps;   /* tuning */
    int      mMinFocusRoiPatches;       /* tuning */

    /* VCM units per diopter, derived at construction from the
     * calibrated VCM range and SensorTuning's
     * module.min_focus_distance_diopters. Zero on fixed-focus or
     * tuning-less sensors — manual-focus diopter conversion then
     * keeps the lens at infinity. */
    float    mVcmPerDiopter;

    mocha::camera::AfMode  mAfMode;
    int32_t  mFocusPosition;

    /* Sweep state — only meaningful when mState != Idle. */
    ScanState mState;
    int32_t   mSweepPos;      /* current / next VCM target           */
    int32_t   mSweepStep;     /* signed: positive toward macro       */
    int32_t   mSweepBestPos;
    float     mSweepBestScore;
    int64_t   mLensMovedNs;   /* systemTime of the last lens move */
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
     * `mMinFramesBetweenSweeps` of the previous one. Stops the
     * controller from cycle-locking on a moving / jittering scene
     * where each retrigger triggers the next. */
    uint32_t  mFramesSinceLastSweep;

    /* AF region of interest in patch coordinates, written by
     * onSettings on the capture engine and read by the stats path on
     * its own thread. Default (full-frame request settings) maps to
     * the centre 8×8 patches via FOCUS_ROI_* constants. */
    mutable std::mutex mFocusRoiMutex;
    FocusRoi  mFocusRoi;
};

}; /* namespace android */

#endif /* HAL_3A_AUTO_FOCUS_CONTROLLER_H */
