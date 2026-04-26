#define LOG_TAG "Cam-SensorTuning"
#include <utils/Log.h>

#include <ctype.h>
#include <fstream>
#include <string.h>

#include <json/json.h>

#include "SensorTuning.h"

namespace android {

namespace {

constexpr const char *kTuningDir = "/vendor/etc/camera/tuning/";

/* Build the filename by convention: `<lower(sensor)>_<lower(integrator)><suffix>.json`.
 * Adding a new module to the HAL means dropping a JSON into the vendor dir —
 * no code change. To try a different tuning branch (e.g. lfi_v3.09 for
 * IMX179), overwrite the file; provenance lives inside the JSON's
 * `sensor.nvidia_version`. The `suffix` slot covers companion files like
 * `_overrides` that share the same per-sensor naming. */
std::string filenameFor(const char *sensor, const char *integrator,
                        const char *suffix = "") {
    std::string s = sensor ? sensor : "";
    std::string i = integrator ? integrator : "";
    for (auto &c : s) c = (char)tolower((unsigned char)c);
    for (auto &c : i) c = (char)tolower((unsigned char)c);
    return s + "_" + i + suffix + ".json";
}

bool readJson(const std::string &path, Json::Value *out, bool warnOnMissing = true) {
    std::ifstream ifs(path.c_str());
    if (!ifs.is_open()) {
        if (warnOnMissing)
            ALOGW("tuning file not found: %s", path.c_str());
        return false;
    }
    Json::Reader reader;
    if (!reader.parse(ifs, *out)) {
        ALOGE("tuning JSON parse failed (%s): %s",
              path.c_str(), reader.getFormattedErrorMessages().c_str());
        return false;
    }
    return true;
}

/* Deep-merge `src` over `dst`. Recurse on object keys; replace
 * everything else (arrays, scalars). Lets a partial overrides JSON
 * sit on top of the regenerated-from-.isp base so HAL-side hand
 * additions survive a full tuning regen. */
void mergeJsonDeep(Json::Value &dst, const Json::Value &src) {
    if (!src.isObject() || !dst.isObject()) {
        dst = src;
        return;
    }
    const Json::Value::Members keys = src.getMemberNames();
    for (const std::string &key : keys) {
        if (dst.isMember(key) && dst[key].isObject() && src[key].isObject()) {
            mergeJsonDeep(dst[key], src[key]);
        } else {
            dst[key] = src[key];
        }
    }
}

} /* namespace */

SensorTuning::SensorTuning()
    : mLoaded(false)
    , mHasAf(false)
    , mModule{{0.0f, 0.0f}, 0.0f, 0.0f}
    , mAf{0, 0, 0, 0, 0, 0, false}
    , mOpticalBlack{0, 0, 0, 0}
    , mAwbRefs{0, 0, 0, 0} {}

SensorTuning::~SensorTuning() = default;

bool SensorTuning::load(const char *sensor, const char *integrator) {
    mLoaded = false;
    mHasAf = false;
    mCcmSets.clear();

    std::string fn = filenameFor(sensor, integrator);
    Json::Value root;
    if (!readJson(std::string(kTuningDir) + fn, &root))
        return false;

    /* Optional companion file with HAL-side hand additions
     * (`active.*` knobs that NVIDIA's stock .isp doesn't carry, e.g.
     * AF state-machine tunables). Merged before parsing so the rest
     * of this function sees one combined tree. Absent file → no-op. */
    {
        const std::string overridesFn = filenameFor(sensor, integrator, "_overrides");
        Json::Value overrides;
        if (readJson(std::string(kTuningDir) + overridesFn, &overrides,
                     /*warnOnMissing=*/false)) {
            mergeJsonDeep(root, overrides);
            ALOGI("tuning overrides applied: %s", overridesFn.c_str());
        }
    }

    if (root["schema_version"].asInt() != 1) {
        ALOGE("tuning schema version mismatch: expected 1, got %d",
              root["schema_version"].asInt());
        return false;
    }

    const Json::Value &mod = root["module"];
    if (!mod.isObject()) {
        ALOGE("tuning: missing `module` block");
        return false;
    }
    const Json::Value &size = mod["sensor_physical_size_mm"];
    if (size.isArray() && size.size() == 2) {
        mModule.physicalSizeMm[0] = size[0].asFloat();
        mModule.physicalSizeMm[1] = size[1].asFloat();
    }
    mModule.focalLengthMm          = mod["focal_length_mm"].asFloat();
    mModule.minFocusDistanceDiopters = mod["min_focus_distance_diopters"].asFloat();
    mBayerPattern = mod["bayer_pattern"].asString();

    const Json::Value &active = root["active"];
    if (!active.isObject()) {
        ALOGE("tuning: missing `active` block");
        return false;
    }

    const Json::Value &af = active["af"];
    if (af.isObject()) {
        mAf.infPos           = af["inf"].asInt();
        mAf.macroPos         = af["macro"].asInt();
        mAf.infOffset        = af["inf_offset"].asInt();
        mAf.macroOffset      = af["macro_offset"].asInt();
        mAf.macroMax         = af["macro_max"].asInt();
        mAf.settleTimeMs     = af["settle_time"].asInt();
        mAf.moduleCalEnable  = af["module_cal_enable"].asInt() != 0;

        /* HAL-side CDAF knobs. Defaults match the reference values
         * RPi libcamera ships in their per-module tuning; overridable
         * via JSON when a sensor needs different settle / step. */
        mAf.stepCoarse =
            af.get("step_coarse", 25).asInt();
        mAf.stepFine =
            af.get("step_fine", 5).asInt();
        mAf.contrastRatio =
            af.get("contrast_ratio", 0.75f).asFloat();
        mAf.retriggerRatio =
            af.get("retrigger_ratio", 0.75f).asFloat();
        mAf.retriggerDelay =
            af.get("retrigger_delay_frames", 10).asInt();
        mAf.pdafEnabled =
            af.get("pdaf_enabled", false).asBool();
        mAf.settleFramesCoarse =
            af.get("settle_frames_coarse", 2).asInt();
        mAf.settleFramesFine =
            af.get("settle_frames_fine", 1).asInt();
        mHasAf = true;
    }

    const Json::Value &cc = active["colorCorrection"]["Set"];
    if (cc.isArray()) {
        for (Json::ArrayIndex i = 0; i < cc.size(); i++) {
            const Json::Value &s = cc[i];
            CcmSet set{};
            set.cctK = s["cct"].asInt();
            const Json::Value &wb = s["wbGain"];
            if (wb.isArray() && wb.size() == 3) {
                for (int k = 0; k < 3; k++) set.wbGain[k] = wb[k].asFloat();
            }
            const Json::Value &mat = s["ccMatrix"];
            if (mat.isArray() && mat.size() == 3) {
                for (int r = 0; r < 3; r++) {
                    const Json::Value &row = mat[r];
                    if (row.isArray() && row.size() == 3) {
                        for (int c = 0; c < 3; c++)
                            set.ccMatrix[r][c] = row[c].asFloat();
                    }
                }
            }
            mCcmSets.push_back(set);
        }
    }

    const Json::Value &ob = active["opticalBlack"];
    if (ob.isObject()) {
        mOpticalBlack.r  = ob["manualBiasR"].asInt();
        mOpticalBlack.gr = ob["manualBiasGR"].asInt();
        mOpticalBlack.gb = ob["manualBiasGB"].asInt();
        mOpticalBlack.b  = ob["manualBiasB"].asInt();
    }

    const Json::Value &mwb = active["mwbCCT"];
    if (mwb.isObject()) {
        mAwbRefs.cctCloudy       = mwb["cloudy"].asInt();
        mAwbRefs.cctShade        = mwb["shade"].asInt();
        mAwbRefs.cctIncandescent = mwb["incandescent"].asInt();
        mAwbRefs.cctFluorescent  = mwb["fluorescent"].asInt();
    }

    const Json::Value &v4 = active["awb"]["v4"];
    if (v4.isObject()) {
        auto read2 = [](const Json::Value &arr, float out[2]) -> bool {
            if (!arr.isArray() || arr.size() != 2) return false;
            out[0] = arr[0].asFloat();
            out[1] = arr[1].asFloat();
            return true;
        };
        const bool okToCct = read2(v4["UtoCCT"], mAwbParams.uToCct);
        const bool okToU   = read2(v4["CCTtoU"], mAwbParams.cctToU);
        if (okToCct && okToU && v4.isMember("LowU") && v4.isMember("HighU")) {
            mAwbParams.lowU   = v4["LowU"].asFloat();
            mAwbParams.highU  = v4["HighU"].asFloat();
            mAwbParams.loaded = true;
        }
        /* Per-frame thresholds & EMA damping. Read independently of
         * the U↔CCT pair — a tuning may define one axis without the
         * other. Callers check each field against 0 to detect
         * presence. */
        if (v4.isMember("CStatsMinThreshold"))
            mAwbParams.cStatsMinThreshold = v4["CStatsMinThreshold"].asFloat();
        if (v4.isMember("CStatsDarkThreshold"))
            mAwbParams.cStatsDarkThreshold = v4["CStatsDarkThreshold"].asFloat();
        if (v4.isMember("SmoothingWpTrackingFraction"))
            mAwbParams.smoothingWpTrackingFraction =
                v4["SmoothingWpTrackingFraction"].asFloat();

        /* Gray-line soft-clamp LUT. Each row in GrayLineSoftClamp
         * is [U_in, U_out, thickness]; NumGrayLineSoftClampPoints
         * bounds the usable range (the array is padded with
         * trailing zero rows). Two slope parameters extend the map
         * beyond the first and last tabulated point. Thickness is
         * read but not yet used — the piecewise-linear map is
         * enough to bound AWB to calibration territory; the
         * band-softening the thickness enables is a later refine. */
        const Json::Value &gl    = v4["GrayLineSoftClamp"];
        const Json::Value &glN   = v4["NumGrayLineSoftClampPoints"];
        const Json::Value &glSB  = v4["GrayLineSoftClampSlopeBeforeFirstPoint"];
        const Json::Value &glSA  = v4["GrayLineSoftClampSlopeAfterLastPoint"];
        if (gl.isArray() && glN.isNumeric()) {
            int n = glN.asInt();
            if (n < 0) n = 0;
            if (n > AwbParams::kGraylineMaxPoints)
                n = AwbParams::kGraylineMaxPoints;
            int kept = 0;
            for (int i = 0; i < n && i < (int)gl.size(); ++i) {
                const Json::Value &row = gl[i];
                if (!row.isArray() || row.size() < 2) break;
                mAwbParams.graylinePoints[kept].uIn  = row[0].asFloat();
                mAwbParams.graylinePoints[kept].uOut = row[1].asFloat();
                mAwbParams.graylinePoints[kept].thickness =
                    row.size() >= 3 ? row[2].asFloat() : 0.f;
                ++kept;
            }
            mAwbParams.graylineCount = kept;
        }
        if (glSB.isNumeric())
            mAwbParams.graylineSlopeBefore = glSB.asFloat();
        if (glSA.isNumeric())
            mAwbParams.graylineSlopeAfter  = glSA.asFloat();

        /* Pre-compute the cold-start anchor from FusionLights at
         * FusionInitLight. Each FusionLight is [R, GR, GB, B] raw
         * means under one tuned illuminant; the default index is
         * the one NVIDIA ships as "start here". Subtract
         * opticalBlack.r so the RGB lives in the same post-BL
         * domain BasicIpa's stats work in. */
        const Json::Value &lights = v4["FusionLights"];
        const Json::Value &init   = v4["FusionInitLight"];
        if (lights.isArray() && init.isNumeric()) {
            const int idx = init.asInt();
            if (idx >= 0 && idx < (int)lights.size()
             && lights[idx].isArray() && lights[idx].size() == 4) {
                const float R  = lights[idx][0].asFloat();
                const float GR = lights[idx][1].asFloat();
                const float GB = lights[idx][2].asFloat();
                const float B  = lights[idx][3].asFloat();
                const int   bl = mOpticalBlack.r;
                const float fR = (R  > bl) ? (R  - bl) : 0.f;
                const float fG = ((GR + GB) * 0.5f > bl)
                               ? ((GR + GB) * 0.5f - bl) : 0.f;
                const float fB = (B  > bl) ? (B  - bl) : 0.f;
                if (fR > 0.f && fG > 0.f && fB > 0.f) {
                    mAwbParams.defaultRawR = fR;
                    mAwbParams.defaultRawG = fG;
                    mAwbParams.defaultRawB = fB;
                    mAwbParams.blackLevelAssumed = bl;
                    mAwbParams.defaultRawValid = true;
                }
            }
        }
    }

    const Json::Value &ae = active["ae"];
    if (ae.isObject()) {
        const Json::Value &mean = ae["MeanAlg"];
        if (mean.isObject()
         && mean.isMember("HigherTarget")  && mean.isMember("LowerTarget")
         && mean.isMember("ConvergeSpeed")) {
            mAeParams.higherTarget     = mean["HigherTarget"].asFloat();
            mAeParams.lowerTarget      = mean["LowerTarget"].asFloat();
            mAeParams.convergeSpeed    = mean["ConvergeSpeed"].asFloat();
            /* Brightness bounds are optional — callers that want
             * adaptive-by-brightness setpoints need both; scalar
             * setpoint consumers ignore them. */
            if (mean.isMember("HigherBrightness"))
                mAeParams.higherBrightness = mean["HigherBrightness"].asFloat();
            if (mean.isMember("LowerBrightness"))
                mAeParams.lowerBrightness  = mean["LowerBrightness"].asFloat();
            if (mean.isMember("ToleranceIn"))
                mAeParams.toleranceIn  = mean["ToleranceIn"].asFloat();
            if (mean.isMember("ToleranceOut"))
                mAeParams.toleranceOut = mean["ToleranceOut"].asFloat();
            mAeParams.loaded = true;
        }
        if (ae.isMember("MaxFstopDeltaPos"))
            mAeParams.maxFstopDeltaPos = ae["MaxFstopDeltaPos"].asFloat();
        if (ae.isMember("MaxFstopDeltaNeg"))
            mAeParams.maxFstopDeltaNeg = ae["MaxFstopDeltaNeg"].asFloat();
    }

    ALOGD("tuning loaded: %s (%s/%s) — %zu CCM sets, AF=%d, BL=(%d,%d,%d,%d)",
          fn.c_str(), sensor, integrator, mCcmSets.size(), mHasAf,
          mOpticalBlack.r, mOpticalBlack.gr, mOpticalBlack.gb, mOpticalBlack.b);

    mLoaded = true;
    return true;
}

void SensorTuning::ccmForCctQ10(int cctK, int16_t out[9]) const {
    if (mCcmSets.empty()) {
        static const int16_t kIdentity[9] = {
            1024, 0, 0,
            0, 1024, 0,
            0, 0, 1024,
        };
        memcpy(out, kIdentity, sizeof(kIdentity));
        return;
    }

    /* Nearest-CCT pick — proper CCT-based interpolation belongs with the
     * AWB work that estimates scene CCT per-frame. */
    const CcmSet *best = &mCcmSets[0];
    int bestDelta = abs(best->cctK - cctK);
    for (size_t i = 1; i < mCcmSets.size(); i++) {
        int d = abs(mCcmSets[i].cctK - cctK);
        if (d < bestDelta) {
            bestDelta = d;
            best = &mCcmSets[i];
        }
    }
    /* Transpose on write: NVIDIA's .isp stores each ccMatrix row as the
     * INPUT-channel → {R_out, G_out, B_out} contribution vector
     * (column-major from our shader's POV). Our demosaic shader, like
     * most camera ISPs, treats ccm rows as OUTPUT-channel coefficients
     * (`rr = ccm[0]*R + ccm[1]*G + ccm[2]*B`). Passing the matrix
     * verbatim produces a neutral-grey-goes-to-magenta failure because
     * the green row's cross-terms balloon. The old hand-coded Q10
     * matrix in the now-deleted IspCalibration had this transpose baked
     * in silently (comments documented the OUTPUT convention); keep the
     * same semantics here. */
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            float v = best->ccMatrix[c][r] * 1024.0f;
            if (v > 32767.0f) v = 32767.0f;
            if (v < -32768.0f) v = -32768.0f;
            out[r * 3 + c] = (int16_t)(v > 0 ? v + 0.5f : v - 0.5f);
        }
    }
}

void SensorTuning::wbGainForCct(int cctK, float out[3]) const {
    if (mCcmSets.empty()) {
        out[0] = out[1] = out[2] = 1.0f;
        return;
    }

    const CcmSet *best = &mCcmSets[0];
    int bestDelta = abs(best->cctK - cctK);
    for (size_t i = 1; i < mCcmSets.size(); i++) {
        int d = abs(mCcmSets[i].cctK - cctK);
        if (d < bestDelta) {
            bestDelta = d;
            best = &mCcmSets[i];
        }
    }
    out[0] = best->wbGain[0];
    out[1] = best->wbGain[1];
    out[2] = best->wbGain[2];
}

void SensorTuning::defaultWbGain(float out[3]) const {
    if (mAwbParams.defaultRawValid
     && mAwbParams.defaultRawR > 0.f
     && mAwbParams.defaultRawG > 0.f
     && mAwbParams.defaultRawB > 0.f) {
        const float G = mAwbParams.defaultRawG;
        out[0] = G / mAwbParams.defaultRawR;
        out[1] = 1.0f;
        out[2] = G / mAwbParams.defaultRawB;
        return;
    }
    out[0] = out[1] = out[2] = 1.0f;
}

namespace {

/* Write the CcmSet's float ccMatrix into `out` as row-major Q10,
 * applying the same .isp→shader transpose ccmForCctQ10 uses. Kept
 * here so both single-set and blended paths share one code path for
 * the sign-clamp / round-to-nearest. */
void emitCcmQ10(const SensorTuning::CcmSet &s, int16_t out[9]) {
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            float v = s.ccMatrix[c][r] * 1024.0f;
            if (v > 32767.0f)  v = 32767.0f;
            if (v < -32768.0f) v = -32768.0f;
            out[r * 3 + c] = (int16_t)(v > 0 ? v + 0.5f : v - 0.5f);
        }
    }
}

/* Blend two CcmSets' float ccMatrix entries by `wa` (weight on `a`)
 * and 1-wa on `b`, then emit Q10 with the same transpose / clamp /
 * round semantics as emitCcmQ10. */
void emitBlendedCcmQ10(const SensorTuning::CcmSet &a,
                       const SensorTuning::CcmSet &b,
                       float wa, int16_t out[9]) {
    const float wb = 1.0f - wa;
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            float v = (wa * a.ccMatrix[c][r] + wb * b.ccMatrix[c][r]) * 1024.0f;
            if (v > 32767.0f)  v = 32767.0f;
            if (v < -32768.0f) v = -32768.0f;
            out[r * 3 + c] = (int16_t)(v > 0 ? v + 0.5f : v - 0.5f);
        }
    }
}

} /* namespace */

int SensorTuning::estimateCctFromU(float U) const {
    if (!mAwbParams.loaded) return 0;
    if (U < mAwbParams.lowU)  U = mAwbParams.lowU;
    if (U > mAwbParams.highU) U = mAwbParams.highU;
    const float cct = mAwbParams.uToCct[0] + mAwbParams.uToCct[1] * U;
    if (cct < 1000.f)  return 1000;
    if (cct > 20000.f) return 20000;
    return (int)(cct + 0.5f);
}

float SensorTuning::clampU(float U) const {
    const int n = mAwbParams.graylineCount;
    if (n <= 0) return U;

    const auto &p = mAwbParams.graylinePoints;

    if (U <= p[0].uIn)
        return p[0].uOut + mAwbParams.graylineSlopeBefore * (U - p[0].uIn);

    for (int i = 0; i < n - 1; ++i) {
        if (U >= p[i].uIn && U <= p[i + 1].uIn) {
            const float span = p[i + 1].uIn - p[i].uIn;
            if (span <= 0.f) return p[i].uOut;
            const float t = (U - p[i].uIn) / span;
            return p[i].uOut + t * (p[i + 1].uOut - p[i].uOut);
        }
    }

    return p[n - 1].uOut
         + mAwbParams.graylineSlopeAfter * (U - p[n - 1].uIn);
}

void SensorTuning::ccmForCctLerpQ10(int estCctK, int16_t out[9]) const {
    if (mCcmSets.empty()) {
        static const int16_t identityQ10[9] = {
            1024, 0, 0,
            0, 1024, 0,
            0, 0, 1024,
        };
        memcpy(out, identityQ10, sizeof(identityQ10));
        return;
    }

    if (mCcmSets.size() == 1) {
        emitCcmQ10(mCcmSets[0], out);
        return;
    }

    /* Find the two CcmSets whose cctK brackets estCctK: the largest
     * cctK <= estCctK (lo) and the smallest cctK >= estCctK (hi).
     * Scene outside the bracket range — clamp to the nearer endpoint
     * so extreme lighting (sodium-vapour, far UV) can't extrapolate
     * a CCM that never existed in calibration. */
    const CcmSet *lo = nullptr;
    const CcmSet *hi = nullptr;
    for (size_t i = 0; i < mCcmSets.size(); ++i) {
        const CcmSet &s = mCcmSets[i];
        if (s.cctK <= estCctK) {
            if (!lo || s.cctK > lo->cctK) lo = &s;
        }
        if (s.cctK >= estCctK) {
            if (!hi || s.cctK < hi->cctK) hi = &s;
        }
    }

    if (!lo) { emitCcmQ10(*hi, out); return; } /* scene cooler than coldest set */
    if (!hi) { emitCcmQ10(*lo, out); return; } /* scene warmer than warmest set */
    if (lo == hi) { emitCcmQ10(*lo, out); return; } /* exact match */

    /* LERP weight in Kelvin. wa is the weight on `lo` (warmer end):
     * estCctK == lo->cctK → wa=1; estCctK == hi->cctK → wa=0. */
    const float span = (float)(hi->cctK - lo->cctK);
    const float wa   = (float)(hi->cctK - estCctK) / span;
    emitBlendedCcmQ10(*lo, *hi, wa, out);
}

}; /* namespace android */
