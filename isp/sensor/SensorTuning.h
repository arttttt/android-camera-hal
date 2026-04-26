#ifndef SENSOR_TUNING_H
#define SENSOR_TUNING_H

#include <stdint.h>
#include <string>
#include <vector>

namespace android {

/* Runtime-loadable per-module tuning. Source of truth: JSON files
 * under /vendor/etc/camera/tuning/, generated from stock NVIDIA .isp
 * overrides via tools/isp_to_json.py.
 *
 * On load failure (file missing / malformed / wrong schema version)
 * the instance stays `!isLoaded()`; consumers should fall back to
 * compile-time defaults and log a warning. No exceptions thrown.
 *
 * Only the `active` JSON section is parsed today. Keys move out of
 * `reserved` into `active` — and into this class's accessors — as
 * the corresponding shader stages ship. */
class SensorTuning {
public:
    struct ModuleInfo {
        float physicalSizeMm[2];         /* {width, height} */
        float focalLengthMm;
        float minFocusDistanceDiopters;  /* 0 for fixed-focus cameras */
    };

    struct AfParams {
        int  infPos;
        int  macroPos;
        int  infOffset;
        int  macroOffset;
        int  macroMax;
        int  settleTimeMs;
        bool moduleCalEnable;

        /* Contrast-detect AF tunables. NVIDIA's stock .isp does not
         * carry these — they're HAL-side knobs, defaulted to RPi
         * libcamera's working values (see reference_af_projects.md)
         * and overridable per-sensor via `active.af.*` JSON keys. */
        int   stepCoarse;       /* coarse sweep step (VCM units)        */
        int   stepFine;         /* fine refinement step (VCM units)     */
        float contrastRatio;    /* sharpness drop fraction that confirms
                                 * the peak has been passed             */
        float retriggerRatio;   /* scene-change threshold; sharpness or
                                 * RGB mean differing from the
                                 * post-converge snapshot by more than
                                 * this fraction starts the
                                 * re-trigger countdown                  */
        int   retriggerDelay;   /* frames the change must persist before
                                 * a continuous-AF re-trigger fires      */

        /* PDAF gate. None of the sensors we ship now have phase
         * pixels (IMX179 / OV5693 / IMX219 are all pre-PDAF Bayer),
         * so the `Pdaf` state in AutoFocusController is a reserved
         * future-work slot. Setting `pdafEnabled=true` without
         * matching kernel + sensor support is a no-op + warning. */
        bool  pdafEnabled;

        /* Per-step VCM settle waits, in frames. The lens isn't a
         * closed-loop motor — V4L2 just relays the position command
         * and the driver has no feedback on when the lens actually
         * arrives. So we wait by frame count instead, with separate
         * values for coarse and fine steps because a 25-unit jump
         * needs noticeably longer to physically settle than a
         * 5-unit nudge. NVIDIA's tuning ships only `settle_time` in
         * milliseconds (which always rounds to one frame at 30 fps)
         * and doesn't distinguish step magnitudes; these knobs are
         * HAL-side additions defaulted to working values when the
         * JSON omits them. */
        int settleFramesCoarse;
        int settleFramesFine;
    };

    struct CcmSet {
        int   cctK;
        float wbGain[3];         /* R, G, B */
        float ccMatrix[3][3];    /* row-major */
    };

    struct OpticalBlack {
        int r, gr, gb, b;
    };

    struct AwbRefs {
        int cctCloudy, cctShade, cctIncandescent, cctFluorescent;
    };

    /* Fusion-AWB calibration from the .isp's awb.v4 block, promoted
     * into the HAL-consumable active section. `loaded` lets callers
     * detect tunings that predate this section and fall back to
     * their pinned-CCT behaviour for the calibration half.
     *
     * CCT axis:
     *   CCT = uToCct[0] + uToCct[1] * U, with U = ln(meanG/meanB)
     *   from raw Bayer. `lowU` / `highU` clamp U before the linear
     *   fit so far-off chromaticities don't extrapolate CCT beyond
     *   the fit's calibrated range.
     *
     * Stats / smoothing axis (consumed by BasicIpa's AWB gate):
     *   cStatsMinThreshold     — per-channel floor for a patch to
     *                            be counted in gray-world (noise
     *                            floor — NVIDIA ships 0.02).
     *   cStatsDarkThreshold    — whole-scene mean-luma floor below
     *                            which AWB holds last state
     *                            (dark-scene pump suppression).
     *   smoothingWpTrackingFraction — AWB gain EMA damping
     *                            (fraction of new toward old). */
    struct AwbParams {
        bool  loaded;
        float uToCct[2];
        float cctToU[2];
        float lowU;
        float highU;
        float cStatsMinThreshold;
        float cStatsDarkThreshold;
        float smoothingWpTrackingFraction;

        /* Post-optical-black raw RGB at the tuning's FusionInitLight
         * entry — NVIDIA's calibrated "start here" illuminant. G is
         * the average of GR and GB. defaultRawValid stays false when
         * the tuning has no FusionLights / FusionInitLight; callers
         * fall back to sensor-neutral (1, 1, 1) priors then.
         * blackLevelAssumed records the opticalBlack.r used for the
         * subtract so the caller knows the reference. */
        bool  defaultRawValid;
        float defaultRawR;
        float defaultRawG;
        float defaultRawB;
        int   blackLevelAssumed;

        /* AWB chromaticity soft-clamp. NVIDIA's gray-line-soft-clamp
         * is a piecewise-linear 1D map from measured U to a clamped
         * U, with linear slope extensions beyond the first and last
         * tabulated point. Applied to the U value before it feeds
         * UtoCCT / gain compute so noisy dark-scene U never drives
         * AWB into territory the sensor was never calibrated for.
         * graylineCount == 0 disables the clamp (legacy tuning). */
        static constexpr int kGraylineMaxPoints = 16;
        struct GraylinePoint {
            float uIn;
            float uOut;
            float thickness;     /* tuning carries this but we
                                  * apply only piecewise-linear map
                                  * for now — room to refine later */
        };
        int   graylineCount;
        GraylinePoint graylinePoints[kGraylineMaxPoints];
        float graylineSlopeBefore;
        float graylineSlopeAfter;

        AwbParams()
            : loaded(false),
              cctToU{0,0}, lowU(0.f), highU(0.f),
              cStatsMinThreshold(0.f),
              cStatsDarkThreshold(0.f),
              smoothingWpTrackingFraction(0.f),
              defaultRawValid(false),
              defaultRawR(0.f), defaultRawG(0.f), defaultRawB(0.f),
              blackLevelAssumed(0),
              graylineCount(0),
              graylineSlopeBefore(0.f),
              graylineSlopeAfter(0.f) {
            uToCct[0] = 0.f; uToCct[1] = 0.f;
            for (int i = 0; i < kGraylineMaxPoints; ++i) {
                graylinePoints[i].uIn      = 0.f;
                graylinePoints[i].uOut     = 0.f;
                graylinePoints[i].thickness = 0.f;
            }
        }
    };

    SensorTuning();
    ~SensorTuning();

    /* Parse JSON at `/vendor/etc/camera/tuning/<lower(sensor)>_<lower(integrator)>.json`.
     * Returns true iff fully parsed; false otherwise, with `isLoaded()`
     * staying false. Safe to call multiple times — replaces any previous
     * state. Adding a new module to the HAL is just dropping a JSON into
     * the vendor dir; no code change. */
    bool load(const char *sensor, const char *integrator);

    bool        isLoaded()      const { return mLoaded; }
    bool        hasAf()         const { return mHasAf; }
    const char *bayerPattern()  const { return mBayerPattern.c_str(); }

    /* NVIDIA ae.MeanAlg — the mean-luma AE controller parameters.
     * Two target/brightness pairs define an adaptive setpoint (dim
     * scenes target a higher luma, bright scenes target a slightly
     * lower one), ConvergeSpeed is the EMA damping on the per-frame
     * correction, and MaxFstopDelta{Pos,Neg} cap the single-frame
     * exposure change in f-stops. `loaded` detects the promoted
     * section; tunings without it let BasicIpa fall back to its
     * compile-time defaults.
     *
     * Target / brightness are in NVIDIA's 0..255 mean-luma scale;
     * the HAL divides by 255 for the [0, 1] pre-gamma domain its
     * histogram stats report. */
    struct AeParams {
        bool  loaded;
        float higherTarget;
        float lowerTarget;
        float higherBrightness;
        float lowerBrightness;
        float convergeSpeed;
        float toleranceIn;      /* dead-band (stops) around setpoint */
        float toleranceOut;     /* exit-dead-band threshold (hysteresis) */
        float maxFstopDeltaPos;
        float maxFstopDeltaNeg;
        AeParams()
            : loaded(false),
              higherTarget(0.f), lowerTarget(0.f),
              higherBrightness(0.f), lowerBrightness(0.f),
              convergeSpeed(0.f),
              toleranceIn(0.f), toleranceOut(0.f),
              maxFstopDeltaPos(0.f), maxFstopDeltaNeg(0.f) {}
    };

    /* Lens-shading correction tuning. Each ctrlPoint carries 10
     * samples per channel (R / GR / GB / B) calibrated at one
     * illuminant CCT. The spatial mapping of those 10 samples (1D
     * edge-to-edge scan, radial profile, or separable X / Y) is the
     * consumer's interpretation — this struct is faithful storage of
     * NVIDIA's layout, not a decoded 2D map. Falloff attenuates the
     * correction strength at high analog gain so noise floor in the
     * corners doesn't get amplified along with the signal. Patch
     * factors are exposed for AE / AWB consumers that may weight
     * patches differently after LSC; not consumed by the shader. */
    struct LscControlPoint {
        static constexpr int kProfileSamples = 10;
        int   cctK;
        int   lightFamily;
        float controlPointR [kProfileSamples];
        float controlPointGR[kProfileSamples];
        float controlPointGB[kProfileSamples];
        float controlPointB [kProfileSamples];
    };

    struct LscFalloffPoint {
        float gain;
        float fadePercent;
    };

    struct LscData {
        bool  loaded;
        bool  moduleCalEnable;
        int   imageWidth;
        int   imageHeight;
        int   ctrlPointsCount;
        std::vector<LscControlPoint> ctrlPoints;

        int   falloffPointsCount;
        std::vector<LscFalloffPoint> falloffPreview;
        std::vector<LscFalloffPoint> falloffStill;
        std::vector<LscFalloffPoint> falloffVideo;

        float leftPatchFactor;
        float centerPatchFactor;
        float topPatchFactor;
        float middlePatchFactor;

        LscData()
            : loaded(false), moduleCalEnable(false),
              imageWidth(0), imageHeight(0), ctrlPointsCount(0),
              falloffPointsCount(0),
              leftPatchFactor(0.f), centerPatchFactor(0.f),
              topPatchFactor(0.f), middlePatchFactor(0.f) {}
    };

    const ModuleInfo&           module()        const { return mModule; }
    const AfParams&             af()            const { return mAf; }
    const std::vector<CcmSet>&  ccmSets()       const { return mCcmSets; }
    const OpticalBlack&         opticalBlack()  const { return mOpticalBlack; }
    const AwbRefs&              awbRefs()       const { return mAwbRefs; }
    const AwbParams&            awbParams()     const { return mAwbParams; }
    const AeParams&             aeParams()      const { return mAeParams; }
    const LscData&              lscData()       const { return mLscData; }

    /* Fill `out` (9 entries, row-major 3x3) with the Q10 fixed-point CCM
     * closest to `cctK`. Uses nearest-CCT from ccmSets(); if the tuning
     * has no sets, writes the identity matrix (1024, 0, 0, 0, 1024, ...).
     * Call sites keep the returned buffer alive for the consumer (the
     * Vulkan ISP stores the pointer). */
    void ccmForCctQ10(int cctK, int16_t out[9]) const;

    /* Fill `out` (3 entries: R, G, B) with the float WB prior gains from
     * the nearest-CCT CcmSet to `cctK`. These are the per-sensor neutral
     * priors NVIDIA ships in the .isp for that illuminant — a raw-domain
     * gray-world AWB that starts from unity drifts away from a neutral
     * white if the sensor's native channel response isn't balanced; the
     * priors are the point the AWB should decay back to in the absence
     * of better info. If the tuning has no sets, writes {1.0, 1.0, 1.0}
     * so callers get a safe unity fallback. */
    void wbGainForCct(int cctK, float out[3]) const;

    /* Fill `out` (R, G, B) with the cold-start WB gains the shader
     * should apply before any AWB tick has landed. Source is
     * awb.v4.FusionLights[FusionInitLight], NVIDIA's calibrated
     * default illuminant in post-optical-black raw RGB. Returned
     * values are *G-unity-normalised shader gains* — i.e. the
     * multiplier each channel needs to reach G_raw:
     *   out[0] = G_raw / R_raw
     *   out[1] = 1.0
     *   out[2] = G_raw / B_raw
     * which is exactly the gray-world result BasicIpa's per-frame
     * AWB produces, so cold-start and converged steady-state live
     * in the same domain and there's no jump at the first tick.
     * Falls back to {1, 1, 1} when the tuning has no FusionLights
     * section or the chosen entry has a zero channel after BL
     * subtract — caller gets unity priors in that case. */
    void defaultWbGain(float out[3]) const;

    /* Estimate scene CCT (Kelvin) from a raw-Bayer chromaticity U.
     * Callers usually pass U = ln(meanG / meanB) = ln(lastWbB) from
     * the gray-world AWB output. Clamps U into [lowU, highU] so
     * far-off chromaticities don't extrapolate beyond the tuned
     * range. Returns 0 when awbParams() isn't loaded — callers should
     * treat that as "no estimate available" and fall back. */
    int  estimateCctFromU(float U) const;

    /* Apply NVIDIA's gray-line soft clamp to a measured chromaticity
     * U: piecewise-linear interpolation between the tabulated
     * (U_in, U_out) points, linear slope extensions beyond the first
     * and last. Upstream of CCT estimation / WB gain compute this
     * bounds AWB to the region the sensor is actually calibrated
     * over — noisy dark-scene U that would otherwise drive both
     * branches out of their trusted range snaps to the nearest
     * on-locus value instead. Pass-through when the tuning has no
     * graylineCount > 0. */
    float clampU(float U) const;

    /* Fill `out` (9 entries, row-major Q10) with the CCM for a scene
     * whose estimated CCT is `estCctK`, linearly interpolating in
     * Kelvin between the two CcmSets whose cctK brackets the estimate.
     *
     * Outside the CcmSet CCT range, clamps to the endpoint's CCM
     * (so very warm or very cool scenes don't extrapolate into
     * nonsense).
     *
     * Fallback behaviour:
     *   - 0 sets: identity (1024, 0, 0, 0, 1024, 0, 0, 0, 1024).
     *   - 1 set: that set's CCM directly.
     *   - 2+ sets: sort by cctK, find the bracket, LERP.
     *
     * Row/column convention matches ccmForCctQ10 (transposed on write
     * from the NVIDIA .isp layout to the shader's OUTPUT-row layout). */
    void ccmForCctLerpQ10(int estCctK, int16_t out[9]) const;

private:
    bool mLoaded;
    bool mHasAf;
    ModuleInfo   mModule;
    AfParams     mAf;
    std::vector<CcmSet> mCcmSets;
    OpticalBlack mOpticalBlack;
    AwbRefs      mAwbRefs;
    AwbParams    mAwbParams;
    AeParams     mAeParams;
    LscData      mLscData;
    std::string  mBayerPattern;
};

}; /* namespace android */

#endif /* SENSOR_TUNING_H */
