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

/* Accept exposures within this envelope after EV compensation. The
 * sensor typically supports longer, but long exposures stall the
 * preview pipeline. */
constexpr int32_t kMinExposureUs = 100;
constexpr int32_t kMaxExposureUs = 200000;

/* EV compensation step: each stop scales exposure by 5/4 up or 4/5
 * down. Matches the ANDROID_CONTROL_AE_COMPENSATION_STEP = 1/3
 * advertised in static characteristics (three 5/4 steps ≈ √2 factor,
 * i.e. one stop). */
constexpr int32_t kEvStepUpNum     = 5;
constexpr int32_t kEvStepUpDenom   = 4;
constexpr int32_t kEvStepDownNum   = 4;
constexpr int32_t kEvStepDownDenom = 5;

/* 1.0x in the gain * extraGainQ8 calculation. */
constexpr int32_t kGainUnityQ8 = 256;

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
        if (evComp > 0) {
            for (int i = 0; i < evComp; i++)
                exposureUs = exposureUs * kEvStepUpNum / kEvStepUpDenom;
        } else {
            for (int i = 0; i < -evComp; i++)
                exposureUs = exposureUs * kEvStepDownNum / kEvStepDownDenom;
        }
    }

    if (exposureUs < kMinExposureUs) exposureUs = kMinExposureUs;
    if (exposureUs > kMaxExposureUs) exposureUs = kMaxExposureUs;

    /* Split target exposure into (actual exposure, extra gain) so a
     * long exposure doesn't drop FPS below the current envelope. */
    int32_t actualExposure;
    int32_t extraGainQ8;
    mCfg.splitExposureGain(exposureUs, &actualExposure, &extraGainQ8);
    mDev->setControl(V4L2_CID_EXPOSURE, actualExposure);

    int32_t gain = mCfg.gainUnit; /* 1x baseline */
    if (cm.exists(ANDROID_SENSOR_SENSITIVITY))
        gain = mCfg.isoToGain(*cm.find(ANDROID_SENSOR_SENSITIVITY).data.i32);
    gain = (int32_t)((int64_t)gain * extraGainQ8 / kGainUnityQ8);
    if (gain < 1)            gain = 1;
    if (gain > mCfg.gainMax) gain = mCfg.gainMax;
    mDev->setControl(V4L2_CID_GAIN, gain);

    /* V4L2_CID_EXPOSURE is fed microseconds directly (splitExposureGain
     * outputs outExposureUs in µs); the report just echoes the value
     * that went to the driver. Earlier revisions multiplied by
     * lineTimeUs which was a bug — applyDefaults stored the raw µs
     * value, so this path now matches. */
    mAppliedExposureUs = actualExposure;
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
