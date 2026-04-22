#ifndef CAMERA_H
#define CAMERA_H

#include <memory>

#include <utils/Errors.h>
#include <hardware/camera_common.h>
#include "V4l2Device.h"
#include <hardware/camera3.h>
#include <camera/CameraMetadata.h>
#include <utils/Mutex.h>

#include "sensor/SensorConfig.h"
#include "sensor/SensorTuning.h"
#include "sensor/DelayedControls.h"
#include "ipa/Ipa.h"
#include "ipa/StatsWorker.h"
#include "IspPipeline.h"
#include "3a/AutoFocusController.h"
#include "3a/ExposureControl.h"
#include "jpeg/JpegEncoder.h"
#include "pipeline/BufferProcessor.h"
#include "pipeline/StreamConfig.h"
#include "DbgUtils.h"

namespace android {

struct PipelineContext;
class Pipeline;
class PipelineStage;
class InFlightTracker;
class RequestThread;
class PipelineThread;
class BayerSource;
template <typename T> class EventQueue;

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
    virtual int registerStreamBuffers(const camera3_stream_buffer_set_t *bufferSet);
    virtual int processCaptureRequest(camera3_capture_request_t *request);

    /* HELPERS/SUBPROCEDURES */

    void notifyShutter(uint32_t frameNumber, uint64_t timestamp);
    void notifyError(uint32_t frameNumber, camera3_stream_t *stream, int errorCode);
    void processCaptureResult(uint32_t frameNumber, const camera_metadata_t *result, const Vector<camera3_stream_buffer> &buffers);

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
    ExposureControl *mExposure;
    JpegEncoder *mJpeg;
    BufferProcessor *mBufferProcessor;
    Mutex mMutex;

    std::unique_ptr<BayerSource>                  mBayerSource;
    std::unique_ptr<InFlightTracker>              mTracker;
    std::unique_ptr<EventQueue<PipelineContext*>> mRequestQueue;
    std::unique_ptr<EventQueue<PipelineContext*>> mPipelineQueue;
    std::unique_ptr<Pipeline>                     mRequestPipeline;
    std::unique_ptr<PipelineStage>                mDemosaicBlitStage;
    std::unique_ptr<PipelineStage>                mStatsProcessStage;
    std::unique_ptr<PipelineStage>                mResultDispatchStage;
    std::unique_ptr<RequestThread>                mRequestThread;
    std::unique_ptr<PipelineThread>               mPipelineThread;
    std::unique_ptr<Ipa>                          mIpa;
    std::unique_ptr<DelayedControls>              mDelayedControls;
    std::unique_ptr<StatsWorker>                  mStatsWorker;
    bool                                          mInfrastructureBuilt;

    /* Build the long-lived per-camera infrastructure (ISP, 3A,
     * BufferProcessor, BayerSource, tracker, pipeline, worker
     * thread). Called lazily from openDevice on first open; survives
     * close/reopen cycles and is torn down only in the destructor. */
    void buildInfrastructure();
    void destroyInfrastructure();

    /* Stop / (re)start the capture and request workers around a
     * configure_streams boundary. Both are idempotent; starting a
     * worker that's already running is a no-op. */
    void stopWorkers();
    void startWorkers();

    /* Pull every PipelineContext still parked in mTracker (orphaned
     * upstream of PipelineThread's own in-flight drain during a
     * stopWorkers), tag each as CAMERA3_MSG_ERROR_REQUEST, and route
     * through ResultDispatchStage so the framework gets its notify()
     * + buffer release. Must run after stopWorkers, before the next
     * configureStreams or closeDevice teardown step. */
    void errorCompletePendingRequests();

    /* STATIC WRAPPERS */

    static int sClose(hw_device_t *device);
    static int sInitialize(const struct camera3_device *device, const camera3_callback_ops_t *callback_ops);
    static int sConfigureStreams(const struct camera3_device *device, camera3_stream_configuration_t *stream_list);
    static int sRegisterStreamBuffers(const struct camera3_device *device, const camera3_stream_buffer_set_t *buffer_set);
    static const camera_metadata_t * sConstructDefaultRequestSettings(const struct camera3_device *device, int type);
    static int sProcessCaptureRequest(const struct camera3_device *device, camera3_capture_request_t *request);
    static void sGetMetadataVendorTagOps(const struct camera3_device *device, vendor_tag_query_ops_t* ops);
    static void sDump(const struct camera3_device *device, int fd);
    static int sFlush(const struct camera3_device *device);

    static camera3_device_ops_t sOps;
};

}; /* namespace android */

#endif // CAMERA_H
