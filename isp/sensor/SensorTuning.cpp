#define LOG_TAG "Cam-SensorTuning"
#include <utils/Log.h>

#include <fstream>
#include <string.h>
#include <strings.h>

#include <json/json.h>

#include "SensorTuning.h"

namespace android {

namespace {

constexpr const char *kTuningDir = "/vendor/etc/camera/tuning/";

/* Fixed (sensor, integrator) → filename lookup. Swap the version in the
 * filename to try an alternative tuning branch (e.g. lfi_v3.09 for IMX179).
 * Adding a new module here is the entire device-porting surface for
 * tuning purposes. */
struct FilenameEntry {
    const char *sensor;
    const char *integrator;
    const char *filename;
};

const FilenameEntry kFilenames[] = {
    { "IMX179", "Primax", "imx179_primax_v2.27.json" },
    { "OV5693", "Sunny",  "ov5693_sunny_v2.13.json"  },
};

bool ieq(const char *a, const char *b) {
    return a && b && strcasecmp(a, b) == 0;
}

bool readJson(const std::string &path, Json::Value *out) {
    std::ifstream ifs(path.c_str());
    if (!ifs.is_open()) {
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

} /* namespace */

SensorTuning::SensorTuning()
    : mLoaded(false)
    , mHasAf(false)
    , mModule{{0.0f, 0.0f}, 0.0f, 0.0f}
    , mAf{0, 0, 0, 0, 0, 0, false}
    , mOpticalBlack{0, 0, 0, 0}
    , mAwbRefs{0, 0, 0, 0} {}

SensorTuning::~SensorTuning() = default;

const char *SensorTuning::filenameFor(const char *sensor, const char *integrator) {
    for (const auto &e : kFilenames) {
        if (ieq(e.sensor, sensor) && ieq(e.integrator, integrator))
            return e.filename;
    }
    return nullptr;
}

bool SensorTuning::load(const char *sensor, const char *integrator) {
    mLoaded = false;
    mHasAf = false;
    mCcmSets.clear();

    const char *fn = filenameFor(sensor, integrator);
    if (!fn) {
        ALOGW("no tuning file registered for %s/%s", sensor, integrator);
        return false;
    }

    Json::Value root;
    if (!readJson(std::string(kTuningDir) + fn, &root))
        return false;

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

    ALOGD("tuning loaded: %s (%s/%s) — %zu CCM sets, AF=%d, BL=(%d,%d,%d,%d)",
          fn, sensor, integrator, mCcmSets.size(), mHasAf,
          mOpticalBlack.r, mOpticalBlack.gr, mOpticalBlack.gb, mOpticalBlack.b);

    mLoaded = true;
    return true;
}

}; /* namespace android */
