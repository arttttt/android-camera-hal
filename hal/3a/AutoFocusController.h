#ifndef HAL_3A_AUTO_FOCUS_CONTROLLER_H
#define HAL_3A_AUTO_FOCUS_CONTROLLER_H

#include <stdint.h>
#include <camera/CameraMetadata.h>

#include "ipa/IpaStats.h"

namespace android {

class V4l2Device;
class IspPipeline;
class SensorTuning;

/* Contrast-detect autofocus for a single-VCM sensor. Drives the V4L2
 * focus subdev and integrates with the ISP AWB lock so the sharpness
 * metric isn't skewed by gain flicker during a sweep. */
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
     * settle time). */
    AutoFocusController(V4l2Device *dev, IspPipeline *isp,
                        const SensorTuning *tuning);

    /* Drive the state machine from request metadata. Reads AF_MODE,
     * LENS_FOCUS_DISTANCE, AF_TRIGGER; also handles the periodic
     * re-trigger in CONTINUOUS_PICTURE mode (gated by frameNumber). */
    void onSettings(const CameraMetadata &cm, uint32_t frameNumber);

    /* If a sweep is active, move the VCM to the current sweep position.
     * Called right after V4L2 dequeue, before CPU/GPU processing. */
    void onFrameStart();

    /* Measure sharpness of the RGBA frame and advance the sweep. When
     * the sweep reaches the end of its range, commits the best VCM
     * position and releases the AWB lock. No-op outside of a sweep. */
    void onFrameData(const uint8_t *rgba, unsigned width, unsigned height);

    /* Advance the sweep against the per-patch sharpness grid for the
     * current frame. Caller hands over the same stats buffer the IPA
     * has already finished with. No-op outside of a sweep. */
    void onSharpnessStats(
        const float sharpness[IpaStats::PATCH_Y][IpaStats::PATCH_X]);

    Report report() const;
    bool   isSweeping() const { return mSweepActive; }

    /* Reset the state-machine to the initial post-construction
     * state. Used at camera close so a stale sweep doesn't leak
     * into the next session. Calibration (VCM positions, settle
     * frames) and the last focus position stay. */
    void reset();

private:
    void startSweep(uint8_t afMode);
    void cancelSweep();

    V4l2Device  *mDev;
    IspPipeline *mIsp;

    /* Calibrated VCM positions — resolved from SensorTuning at
     * construction when available, otherwise compile-time defaults. */
    int32_t  mVcmInfinity;
    int32_t  mVcmMacroStart;
    int32_t  mVcmMacroEnd;
    int32_t  mVcmAutoEnd;
    int32_t  mVcmSettleFrames;

    uint8_t  mAfMode;
    int32_t  mFocusPosition;
    bool     mSweepActive;
    int32_t  mSweepPos;
    int32_t  mSweepStep;
    int32_t  mSweepEnd;
    int32_t  mSweepBestPos;
    uint64_t mSweepBestScore;
    int32_t  mSettleFrames;
};

}; /* namespace android */

#endif /* HAL_3A_AUTO_FOCUS_CONTROLLER_H */
