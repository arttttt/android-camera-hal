#ifndef HAL_3A_EXPOSURE_CONTROL_H
#define HAL_3A_EXPOSURE_CONTROL_H

#include <stdint.h>
#include <camera/CameraMetadata.h>

namespace android {

class V4l2Device;
struct SensorConfig;

/* Per-frame exposure + ISO gain applied to V4L2 sensor controls.
 * Parses ANDROID_SENSOR_EXPOSURE_TIME / SENSOR_SENSITIVITY /
 * AE_EXPOSURE_COMPENSATION, applies EV compensation on top of the
 * request exposure, and splits a long exposure into a (short exposure
 * + extra gain) pair so FPS stays within the current envelope. */
class ExposureControl {
public:
    struct Report {
        int32_t appliedExposureUs;
        int32_t appliedGain;
    };

    ExposureControl(V4l2Device *dev, const SensorConfig &cfg);

    /* Push sensor defaults to V4L2. Called once after configureStreams,
     * before STREAMON. */
    void applyDefaults();

    /* Parse per-frame exposure/gain from request metadata, push to
     * V4L2. Updates the value returned by report(). */
    void onSettings(const CameraMetadata &cm);

    Report report() const;

private:
    V4l2Device         *mDev;
    const SensorConfig &mCfg;
    int32_t             mAppliedExposureUs;
    int32_t             mAppliedGain;
};

}; /* namespace android */

#endif /* HAL_3A_EXPOSURE_CONTROL_H */
