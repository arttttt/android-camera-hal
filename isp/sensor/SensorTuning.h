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

    const ModuleInfo&           module()        const { return mModule; }
    const AfParams&             af()            const { return mAf; }
    const std::vector<CcmSet>&  ccmSets()       const { return mCcmSets; }
    const OpticalBlack&         opticalBlack()  const { return mOpticalBlack; }
    const AwbRefs&              awbRefs()       const { return mAwbRefs; }

    /* Fill `out` (9 entries, row-major 3x3) with the Q10 fixed-point CCM
     * closest to `cctK`. Uses nearest-CCT from ccmSets(); if the tuning
     * has no sets, writes the identity matrix (1024, 0, 0, 0, 1024, ...).
     * Call sites keep the returned buffer alive for the consumer (the
     * Vulkan ISP stores the pointer). */
    void ccmForCctQ10(int cctK, int16_t out[9]) const;

private:
    bool mLoaded;
    bool mHasAf;
    ModuleInfo   mModule;
    AfParams     mAf;
    std::vector<CcmSet> mCcmSets;
    OpticalBlack mOpticalBlack;
    AwbRefs      mAwbRefs;
    std::string  mBayerPattern;
};

}; /* namespace android */

#endif /* SENSOR_TUNING_H */
