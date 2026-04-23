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

        AwbParams()
            : loaded(false),
              cctToU{0,0}, lowU(0.f), highU(0.f),
              cStatsMinThreshold(0.f),
              cStatsDarkThreshold(0.f),
              smoothingWpTrackingFraction(0.f),
              defaultRawValid(false),
              defaultRawR(0.f), defaultRawG(0.f), defaultRawB(0.f),
              blackLevelAssumed(0) {
            uToCct[0] = 0.f; uToCct[1] = 0.f;
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

    const ModuleInfo&           module()        const { return mModule; }
    const AfParams&             af()            const { return mAf; }
    const std::vector<CcmSet>&  ccmSets()       const { return mCcmSets; }
    const OpticalBlack&         opticalBlack()  const { return mOpticalBlack; }
    const AwbRefs&              awbRefs()       const { return mAwbRefs; }
    const AwbParams&            awbParams()     const { return mAwbParams; }
    const AeParams&             aeParams()      const { return mAeParams; }

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
    std::string  mBayerPattern;
};

}; /* namespace android */

#endif /* SENSOR_TUNING_H */
