#ifndef CAMERA_H
#define CAMERA_H

#include <atomic>
#include <functional>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include <utils/Errors.h>
#include <hardware/camera_common.h>
#include "V4l2Device.h"
#include <hardware/camera3.h>
#include "CameraMetadataCompat.h"
#include <utils/Mutex.h>

#include "sensor/SensorConfig.h"
#include "sensor/SensorTuning.h"
#include "sensor/DelayedControls.h"
#include "ipa/Ipa.h"
#include "IspPipeline.h"
#include "3a/AutoFocusController.h"
#include "jpeg/JpegEncoder.h"
#include "pipeline/BufferProcessor.h"
#include "Camera3GrallocPort.h"
#include "mochacamera/Buffers.h"
#include "mochacamera/Capability.h"
#include "mochacamera/Controls.h"
#include "pipeline/StreamConfig.h"
#include "DbgUtils.h"

namespace mocha::camera {
struct Frame;
class FrameRegistry;
class CaptureEngine;
class StatsEngine;
class IspEngine;
class FinishEngine;
class JpegEngine;
class ResultEngine;
}

namespace android {

class Pipeline;
class ApplySettingsStage;
class DemosaicBlitStage;
class StatsProcessStage;
class ResultDispatchStage;
class DeviceErrorReporter;
class Camera3ResultSink;

class Camera: public camera3_device {
public:
    Camera(const char *devNode, int facing);
    virtual ~Camera();

    bool isValid() { return mValid; }

    virtual status_t cameraInfo(struct camera_info *info);

    virtual int openDevice(hw_device_t **device);
    virtual int closeDevice();

protected:
    virtual camera_metadata_t * staticCharacteristics();
    virtual int initialize(const camera3_callback_ops_t *callbackOps);
    virtual int configureStreams(camera3_stream_configuration_t *streamList);
    virtual const camera_metadata_t * constructDefaultRequestSettings(int type);
    virtual int processCaptureRequest(camera3_capture_request_t *request);
    virtual int flush();

    /* HELPERS/SUBPROCEDURES */

    void notifyError(uint32_t frameNumber, camera3_stream_t *stream, int errorCode);

    camera_metadata_t *mStaticCharacteristics;
    camera_metadata_t *mDefaultRequestSettings[CAMERA3_TEMPLATE_COUNT];
    CameraMetadata mLastRequestSettings;

    V4l2Device *mDev;
    bool mValid;
    int mFacing;
    const camera3_callback_ops_t *mCallbackOps;

    size_t mJpegBufferSize;

    bool mSoftIspEnabled;

    SensorConfig mSensorCfg;
    SensorTuning mTuning;
    int16_t      mCcmQ10[9];

private:
    IspPipeline *mIsp;
    AutoFocusController *mAf;
    JpegEncoder *mJpeg;
    BufferProcessor *mBufferProcessor;
    Mutex mMutex;

    /* What the camera can do; the static metadata, the templates and
     * the stream checks read it. Probed on first use. */
    mocha::camera::Capability                     mCapability;
    bool                                          mCapabilityProbed = false;
    const mocha::camera::Capability &capability();
    /* The core's results, to the framework. */
    std::unique_ptr<Camera3ResultSink>            mResultSink;
    Camera3GrallocPort                            mGralloc;
    /* The configured streams, by StreamId (their index here). */
    std::vector<std::pair<camera3_stream_t *, mocha::camera::StreamSpec>> mStreams;
    std::unique_ptr<mocha::camera::FrameSource>   mFrameSource;
    std::unique_ptr<mocha::camera::FrameRegistry> mRegistry;
    std::unique_ptr<DeviceErrorReporter>          mDeviceError;
    /* ApplySettings + ShutterNotify, run by the capture engine. */
    std::unique_ptr<Pipeline>                     mSettingsPipeline;
    /* Owned by mSettingsPipeline; kept to reset it on session close. */
    ApplySettingsStage                           *mApplySettingsStage = nullptr;
    std::unique_ptr<DemosaicBlitStage>            mDemosaicBlitStage;
    std::unique_ptr<StatsProcessStage>            mStatsProcessStage;
    std::unique_ptr<ResultDispatchStage>          mResultDispatchStage;
    std::unique_ptr<Ipa>                          mIpa;
    std::unique_ptr<DelayedControls>              mDelayedControls;
    /* The engines, one thread each; built once, their threads started
     * and stopped around every stream configuration. */
    std::unique_ptr<mocha::camera::ResultEngine>  mResultEngine;
    std::unique_ptr<mocha::camera::JpegEngine>    mJpegEngine;
    std::unique_ptr<mocha::camera::FinishEngine>  mFinishEngine;
    std::unique_ptr<mocha::camera::IspEngine>     mIspEngine;
    std::unique_ptr<mocha::camera::StatsEngine>   mStatsEngine;
    std::unique_ptr<mocha::camera::CaptureEngine> mCaptureEngine;
    bool                                          mEnginesRunning = false;
    bool                                          mInfrastructureBuilt;

    /* V4L2 capture resolution from the last configure_streams. The
     * framework re-issues configure_streams on AE_MODE auto↔manual
     * toggles even when the streams haven't changed; the heavy
     * stopEngines + STREAMOFF + prewarm + STREAMON + startEngines
     * cycle is unnecessary when the V4L2 input resolution is the
     * same. 0 = no prior config (first call must run the full path). */
    unsigned                                      mLastConfigWidth = 0;
    unsigned                                      mLastConfigHeight = 0;

    /* AE_MODE seen on the previous processCaptureRequest. Used to
     * detect manual→auto transitions and cancel the frames still
     * waiting for their Bayer — Camera2 apps generally don't call
     * flush() on the mode change so we can't rely on the framework.
     * Empty until the first request of the session. */
    std::optional<mocha::camera::AeMode>          mLastAeMode;

    /* Set while flush() waits for the HAL to drain; a request arriving
     * then is cancelled at once. */
    std::atomic<bool>                             mFlushing{false};
    static constexpr int                          kFlushTimeoutMs = 1000;

    /* Build the long-lived per-camera infrastructure (ISP, 3A,
     * BufferProcessor, frame source, registry, stages, engines). Called
     * lazily from openDevice on first open; survives close/reopen
     * cycles and is torn down only in the destructor. */
    void buildInfrastructure();
    void destroyInfrastructure();

    /* Pull driver-advertised ranges (frame-length, gain, exposure)
     * into mSensorCfg via VIDIOC_QUERYCTRL. Called from openDevice
     * before buildInfrastructure so every subsequent consumer —
     * Ipa3A's ctor, ApplySettingsStage's manual path, result
     * metadata — reads live V4L2 values rather than SensorConfig's
     * static seeds. */
    void populateSensorConfigFromDriver();

    /* Start / stop the engines' threads around a configure_streams
     * boundary. stopEngines hands every frame still held back to the
     * framework (normally, or as a failed request) before it returns.
     * Both are idempotent. */
    bool startEngines();
    void stopEngines();

    /* The one cancellation flush() and the AE mode switch share:
     * cancel the in-flight frames `which` picks (committed ones
     * complete anyway) and wake a capture waiting on a long exposure.
     * Takes no lock of Camera's own, so it is safe under mMutex. */
    void cancelInFlight(const std::function<bool(const mocha::camera::Frame &)> &which);


    /* STATIC WRAPPERS */

    static int sClose(hw_device_t *device);
    static int sInitialize(const struct camera3_device *device, const camera3_callback_ops_t *callback_ops);
    static int sConfigureStreams(const struct camera3_device *device, camera3_stream_configuration_t *stream_list);
    static const camera_metadata_t * sConstructDefaultRequestSettings(const struct camera3_device *device, int type);
    static int sProcessCaptureRequest(const struct camera3_device *device, camera3_capture_request_t *request);
    static void sDump(const struct camera3_device *device, int fd);
    static int sFlush(const struct camera3_device *device);

    static camera3_device_ops_t sOps;
};

}; /* namespace android */

#endif // CAMERA_H
