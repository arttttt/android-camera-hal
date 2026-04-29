#define LOG_TAG "Cam-AE"

#include "ExposureControl.h"

#include <stdint.h>
#include <linux/videodev2.h>

#include <utils/Log.h>
#include <system/camera_metadata.h>

#include "V4l2Controls.h"
#include "V4l2Device.h"
#include "sensor/SensorConfig.h"

namespace android {

namespace {

/* EV compensation step: each stop scales exposure by 5/4 up or 4/5
 * down. Matches the ANDROID_CONTROL_AE_COMPENSATION_STEP = 1/3
 * advertised in static characteristics (three 5/4 steps ≈ √2 factor,
 * i.e. one stop). */
constexpr int32_t kEvStepUpNum     = 5;
constexpr int32_t kEvStepUpDenom   = 4;
constexpr int32_t kEvStepDownNum   = 4;
constexpr int32_t kEvStepDownDenom = 5;

inline int32_t applyEvComp(int32_t exposureUs, int32_t evComp) {
    if (evComp > 0) {
        for (int i = 0; i < evComp; i++)
            exposureUs = exposureUs * kEvStepUpNum / kEvStepUpDenom;
    } else {
        for (int i = 0; i < -evComp; i++)
            exposureUs = exposureUs * kEvStepDownNum / kEvStepDownDenom;
    }
    return exposureUs;
}

inline int32_t clampInt(int32_t v, int32_t lo, int32_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

} /* namespace */

ExposureControl::ExposureControl(V4l2Device *dev, const SensorConfig &cfg)
    : mDev(dev)
    , mCfg(cfg)
    , mAppliedExposureUs(cfg.exposureDefault)
    , mAppliedGain(cfg.gainDefault) {
}

void ExposureControl::applyDefaults() {
    mDev->setControl(V4L2_CID_EXPOSURE, mCfg.exposureDefault);
    mDev->setControl(V4L2_CID_GAIN,     mCfg.gainDefault);
    mAppliedExposureUs = mCfg.exposureDefault;
    mAppliedGain       = mCfg.gainDefault;
}

void ExposureControl::onSettings(const CameraMetadata &cm) {
    int32_t exposureUs = mCfg.exposureDefault;
    if (cm.exists(ANDROID_SENSOR_EXPOSURE_TIME)) {
        int64_t exposureNs = *cm.find(ANDROID_SENSOR_EXPOSURE_TIME).data.i64;
        exposureUs = (int32_t)(exposureNs / 1000);
    }

    /* EV compensation applied on top of the requested exposure. */
    if (cm.exists(ANDROID_CONTROL_AE_EXPOSURE_COMPENSATION)) {
        int32_t evComp = *cm.find(ANDROID_CONTROL_AE_EXPOSURE_COMPENSATION).data.i32;
        exposureUs = applyEvComp(exposureUs, evComp);
    }

    /* Driver-queried envelope; populateSensorConfigFromDriver fills
     * exposureMin / exposureMax from V4L2_CID_EXPOSURE QUERYCTRL. */
    exposureUs = clampInt(exposureUs, mCfg.exposureMin, mCfg.exposureMax);

    uint8_t aeMode = ANDROID_CONTROL_AE_MODE_ON;
    if (cm.exists(ANDROID_CONTROL_AE_MODE))
        aeMode = *cm.find(ANDROID_CONTROL_AE_MODE).data.u8;

    int32_t gain = mCfg.gainUnit;  /* 1.0x baseline */
    if (cm.exists(ANDROID_SENSOR_SENSITIVITY))
        gain = mCfg.isoToGain(*cm.find(ANDROID_SENSOR_SENSITIVITY).data.i32);

    int32_t actualExposureUs;
    int32_t frameLen;

    if (aeMode == ANDROID_CONTROL_AE_MODE_OFF) {
        /* Manual AE: app told us exactly what to do. Honour the
         * requested exposure verbatim; if it doesn't fit the default
         * frame_length, grow frame_length so it does (FPS drops
         * accordingly — that's the user's choice when they set
         * a 1 s shutter). No splitExposureGain, no fps preservation. */
        actualExposureUs = exposureUs;
        frameLen         = mCfg.frameLenForExposure(exposureUs);
    } else {
        /* Auto AE cold-start fallback (IPA hasn't pushed yet). Hold
         * fps at the default frame_length and trade overflow exposure
         * for extra gain so preview cadence stays smooth. */
        int32_t extraGainQ8;
        mCfg.splitExposureGain(exposureUs, &actualExposureUs, &extraGainQ8);
        gain = (int32_t)((int64_t)gain * extraGainQ8 / mCfg.gainUnit);
        frameLen = mCfg.frameLenDefault;
    }

    gain = clampInt(gain, mCfg.gainMin, mCfg.gainMax);

    /* Single VIDIOC_S_EXT_CTRLS — frame_length must reach the sensor
     * before / together with the exposure that needs it, otherwise the
     * driver clamps the exposure into the old (smaller) frame and the
     * user sees nothing change. */
    V4l2Controls ctrls;
    ctrls.add(V4L2_CID_FRAME_LENGTH, frameLen);
    ctrls.add(V4L2_CID_EXPOSURE,     actualExposureUs);
    ctrls.add(V4L2_CID_GAIN,         gain);
    mDev->setControls(ctrls);

    mAppliedExposureUs = actualExposureUs;
    mAppliedGain       = gain;
}

void ExposureControl::applyBatch(const DelayedControls::Batch &batch) {
    V4l2Controls ctrls;
    if (batch.has[DelayedControls::EXPOSURE]) {
        ctrls.add(V4L2_CID_EXPOSURE, batch.val[DelayedControls::EXPOSURE]);
        mAppliedExposureUs = batch.val[DelayedControls::EXPOSURE];
    }
    if (batch.has[DelayedControls::GAIN]) {
        ctrls.add(V4L2_CID_GAIN, batch.val[DelayedControls::GAIN]);
        mAppliedGain = batch.val[DelayedControls::GAIN];
    }
    if (ctrls.count > 0) {
        mDev->setControls(ctrls);
    }
}

ExposureControl::Report ExposureControl::report() const {
    Report r;
    r.appliedExposureUs = mAppliedExposureUs;
    r.appliedGain       = mAppliedGain;
    return r;
}

}; /* namespace android */
