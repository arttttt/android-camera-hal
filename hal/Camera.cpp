/*
 * Copyright (C) 2015-2016 Antmicro
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define LOG_TAG "Cam-Camera"
#define LOG_NDEBUG NDEBUG

#include <hardware/camera3.h>
#include <camera/CameraMetadata.h>
#include <utils/misc.h>
#include <utils/Log.h>
#include <hardware/gralloc.h>
#include <ui/Rect.h>
#include <ui/GraphicBuffer.h>
#include <ui/GraphicBufferMapper.h>
#include <ui/Fence.h>
#include <assert.h>
#include <math.h>
#include <linux/videodev2.h>
#include <libyuv/scale_argb.h>
#include <libyuv/rotate_argb.h>
#include <cutils/properties.h>
#include <vector>

/* Tegra camera CIDs — not in standard V4L2 headers */
#ifndef V4L2_CID_FRAME_LENGTH
#define V4L2_CID_FRAME_LENGTH   (V4L2_CTRL_CLASS_CAMERA | 0x2000)
#endif

#include "DbgUtils.h"
#include "Camera.h"
#include "metadata/CameraStaticMetadata.h"
#include "metadata/RequestTemplateBuilder.h"
#include "metadata/ResultMetadataBuilder.h"
#include "IspPipeline.h"

#include "BayerSource.h"
#include "V4l2Source.h"
#include "EventQueue.h"
#include "pipeline/CaptureRequest.h"
#include "pipeline/PipelineContext.h"
#include "pipeline/Pipeline.h"
#include "pipeline/InFlightTracker.h"
#include "pipeline/RequestThread.h"
#include "pipeline/PipelineThread.h"
#include "pipeline/stages/ApplySettingsStage.h"
#include "pipeline/stages/ShutterNotifyStage.h"
#include "pipeline/stages/CaptureStage.h"
#include "pipeline/stages/DemosaicBlitStage.h"
#include "pipeline/stages/ResultDispatchStage.h"
#include "pipeline/stages/StatsDispatchStage.h"
#include "pipeline/stages/StatsProcessStage.h"
#include "ipa/BasicIpa.h"

extern camera_module_t HAL_MODULE_INFO_SYM;

namespace {
/* Soft cap; see docs/tier3_architecture.md. */
constexpr size_t REQUEST_QUEUE_CAPACITY = 256;

/* Depth of concurrent GPU submits PipelineThread will keep outstanding.
 * Bounded above by VulkanIspPipeline::SLOT_COUNT (the per-slot ring).
 *
 * Pinned at 1 for single-stream preview. On this hardware GPU is
 * ~60 ms / frame, CPU prep is ~2 ms, sensor cadence is ~33 ms — GPU
 * is the bottleneck and there's nothing CPU-side worth overlapping.
 * Higher depths just add per-frame latency and let completions clump
 * (four ~60 ms submits finish in a batch rather than at sensor
 * cadence). Raise when PR 7 / 8 put real CPU work beside the GPU
 * (IPA stats, JPEG encode). */
constexpr size_t PIPELINE_MAX_IN_FLIGHT = 1;

/* Handoff buffer between RequestThread and PipelineThread. Decoupled
 * from PIPELINE_MAX_IN_FLIGHT on purpose: the queue size adds directly
 * to the number of Bayer slots reserved out of V4L2 at once (each ctx
 * in the queue still owns its acquireNextFrame'd VBuffer).
 *
 * Worst-case V4L2 slot hold per session =
 *   PIPELINE_MAX_IN_FLIGHT    (ctxs PipelineThread has taken on)
 * + PIPELINE_QUEUE_CAPACITY   (ctxs sitting in the handoff queue)
 * + 1                         (V4l2Source::latest, pre-consumer handoff)
 * — so 1 + 1 + 1 = 3 slots worst-case, leaving V4L2DEVICE_BUF_COUNT-3
 * for the sensor. Larger queue caps only buy one-sided bursts at the
 * cost of sensor starvation. */
constexpr size_t PIPELINE_QUEUE_CAPACITY = 1;
} /* namespace */

namespace android {
/**
 * \class Camera
 *
 * Android's Camera 3 device implementation.
 *
 * Declaration of camera capabilities, frame request handling, etc. This code
 * is what Android framework talks to.
 */

Camera::Camera(const char *devNode, int facing)
    : mStaticCharacteristics(NULL)
    , mCallbackOps(NULL)
    , mFacing(facing)
    , mJpegBufferSize(0)
    , mSoftIspEnabled(true)
    , mInfrastructureBuilt(false) {
    DBGUTILS_AUTOLOGCALL(__func__);
    for(size_t i = 0; i < NELEM(mDefaultRequestSettings); i++) {
        mDefaultRequestSettings[i] = NULL;
    }

    common.tag      = HARDWARE_DEVICE_TAG;
    common.version  = CAMERA_DEVICE_API_VERSION_3_0;
    common.module   = &HAL_MODULE_INFO_SYM.common;
    common.close    = Camera::sClose;
    ops             = &sOps;
    priv            = NULL;

    mValid = true;
    mIsp = NULL;
    mAf = NULL;
    mExposure = NULL;
    mJpeg = NULL;
    mBufferProcessor = NULL;

    /* Mi Pad 1 sensor → module mapping. Constructor is the earliest
     * point where facing is known; loading here means the tuning is
     * ready for both staticCharacteristics() (cameraInfo) and
     * configureStreams() (open). */
    const char *sensorName = (mFacing == CAMERA_FACING_BACK) ? "IMX179" : "OV5693";
    const char *integrator = (mFacing == CAMERA_FACING_BACK) ? "Primax" : "Sunny";
    mTuning.load(sensorName, integrator);

    /* Sensor-config (frame-length / gain ranges etc.) is a function
     * of facing alone; load once. Driver-reported limits are merged
     * in later at configureStreams time. */
    mSensorCfg = (mFacing == CAMERA_FACING_BACK) ?
        SensorConfig::imx179() : SensorConfig::ov5693();

    /* Soft-ISP vs (legacy) HW-ISP selector. Read once per camera
     * instance; toggling the property mid-session has no effect. */
    char propVal[PROPERTY_VALUE_MAX] = {0};
    property_get("persist.camera.soft_isp", propVal, "1");
    mSoftIspEnabled = (propVal[0] == '1');

    mDev = new V4l2Device(devNode);
    if(!mDev) {
        mValid = false;
    }
}

Camera::~Camera() {
    DBGUTILS_AUTOLOGCALL(__func__);
    stopWorkers();
    destroyInfrastructure();
    if (mDev) {
        mDev->disconnect();
        delete mDev;
        mDev = NULL;
    }
}

status_t Camera::cameraInfo(struct camera_info *info) {
    DBGUTILS_AUTOLOGCALL(__func__);
    Mutex::Autolock lock(mMutex);
    info->facing = mFacing;
    info->orientation = (mFacing == CAMERA_FACING_FRONT) ? 270 : 90;
    info->device_version = CAMERA_DEVICE_API_VERSION_3_0;
    info->static_camera_characteristics = staticCharacteristics();

    return NO_ERROR;
}

void Camera::populateSensorConfigFromDriver() {
    int32_t flMin, flMax, flDef;
    if (mDev->queryControl(V4L2_CID_FRAME_LENGTH, &flMin, &flMax, &flDef)) {
        mSensorCfg.frameLenDefault = flDef;
        /* Cap at 3× default so AE can't stretch frame_length to
         * 30 kilo-lines in a dark scene; beyond 3× the fps drop is
         * more disruptive than the extra exposure headroom is
         * worth. */
        mSensorCfg.frameLenMax = (flMax > flDef * 3) ? flDef * 3 : flMax;
        ALOGD("Frame length: def=%d max=%d (driver max=%d)",
              mSensorCfg.frameLenDefault, mSensorCfg.frameLenMax, flMax);
    }
    int32_t gMin, gMax, gDef;
    if (mDev->queryControl(V4L2_CID_GAIN, &gMin, &gMax, &gDef)) {
        mSensorCfg.gainMax     = gMax;
        mSensorCfg.gainDefault = gDef;
        ALOGD("Gain: min=%d max=%d def=%d", gMin, gMax, gDef);
    }
    int32_t eMin, eMax, eDef;
    if (mDev->queryControl(V4L2_CID_EXPOSURE, &eMin, &eMax, &eDef)) {
        mSensorCfg.exposureDefault = eDef;
        ALOGD("Exposure: min=%d max=%d def=%d (us)", eMin, eMax, eDef);
    }
}

int Camera::openDevice(hw_device_t **device) {
    DBGUTILS_AUTOLOGCALL(__func__);
    Mutex::Autolock lock(mMutex);
    mDev->connect();
    *device = &common;

    /* Open focuser for back camera */
    if (mFacing == CAMERA_FACING_BACK)
        mDev->openFocuser("/dev/v4l-subdev0");

    /* Pull driver-advertised ranges into mSensorCfg before anything
     * consumes it. BasicIpa's ctor (inside buildInfrastructure) uses
     * gainDefault / exposureDefault / gainMax / frameLenDefault to
     * seed AE state — querying afterwards leaves BasicIpa holding
     * stale values from SensorConfig's static seeds until the first
     * session's configureStreams runs. */
    populateSensorConfigFromDriver();

    /* Lazy first-time build of the long-lived infrastructure. Kept
     * alive across close/reopen cycles; destroyed only in ~Camera. */
    buildInfrastructure();

    return NO_ERROR;
}

int Camera::closeDevice() {
    DBGUTILS_AUTOLOGCALL(__func__);

    /* Stop workers before taking mMutex — a worker may still be
     * finishing its last iteration (which doesn't need the lock but
     * may call framework callbacks). PipelineThread drains its own
     * in-flight contexts on stop; anything still in the tracker is
     * stuck upstream (RequestThread bailed out of pushBlocking) and
     * reaped here. */
    stopWorkers();

    Mutex::Autolock lock(mMutex);

    /* Error-complete any requests that got enqueued but never pushed
     * downstream — RequestThread ran Apply/Shutter/Capture but got
     * unblocked out of pushBlocking during stopWorkers. ResultDispatch
     * alone is enough for cleanup: DemosaicBlit is skipped on error,
     * and the dispatch stage emits notify(ERROR_REQUEST) + releases
     * any Bayer slot the stage chain already took. */
    errorCompletePendingRequests();

    /* Drop per-session state so the next open starts from a clean
     * slate. Infrastructure (ISP core, 3A, BufferProcessor, pipeline,
     * worker threads) survives and will be reused. */
    mLastRequestSettings.clear();
    if (mAf)              mAf->reset();
    if (mIsp)             mIsp->onSessionClose();
    if (mIpa)             mIpa->reset();
    if (mDelayedControls) mDelayedControls->reset();
    if (mStatsWorker)     mStatsWorker->reset();

    mDev->disconnect();
    return NO_ERROR;
}

camera_metadata_t *Camera::staticCharacteristics() {
    if(mStaticCharacteristics)
        return mStaticCharacteristics;
    mStaticCharacteristics = CameraStaticMetadata::build(mDev, mFacing, &mTuning,
                                                          &mJpegBufferSize);
    return mStaticCharacteristics;
}

int Camera::initialize(const camera3_callback_ops_t *callbackOps) {
    DBGUTILS_AUTOLOGCALL(__func__);
    Mutex::Autolock lock(mMutex);

    mCallbackOps = callbackOps;
    return NO_ERROR;
}

const camera_metadata_t * Camera::constructDefaultRequestSettings(int type) {
    DBGUTILS_AUTOLOGCALL(__func__);
    Mutex::Autolock lock(mMutex);
    /* TODO: validate type */

    if(mDefaultRequestSettings[type])
        return mDefaultRequestSettings[type];
    mDefaultRequestSettings[type] = RequestTemplateBuilder::build(type, mDev, mFacing);
    return mDefaultRequestSettings[type];
}

int Camera::configureStreams(camera3_stream_configuration_t *streamList) {
    DBGUTILS_AUTOLOGCALL(__func__);
    Mutex::Autolock lock(mMutex);

    /* TODO: sanity checks */

    ALOGV("+-------------------------------------------------------------------------------");
    ALOGV("| STREAMS FROM FRAMEWORK");
    ALOGV("+-------------------------------------------------------------------------------");
    for(size_t i = 0; i < streamList->num_streams; ++i) {
        camera3_stream_t *newStream = streamList->streams[i];
        ALOGV("| p=%p  fmt=0x%.2x  type=%u  usage=0x%.8x  size=%4ux%-4u  buf_no=%u",
              newStream,
              newStream->format,
              newStream->stream_type,
              newStream->usage,
              newStream->width,
              newStream->height,
              newStream->max_buffers);
    }
    ALOGV("+-------------------------------------------------------------------------------");

    unsigned width  = 0;
    unsigned height = 0;
    {
        status_t e = StreamConfig::normalize(streamList, &width, &height);
        if (e != NO_ERROR)
            return e;
    }

    /* Infrastructure (ISP, 3A, BufferProcessor, BayerSource, pipeline,
     * request thread) was built on the first openDevice and lives for
     * the Camera instance's lifetime. This function only (1) stops
     * the workers, (2) reconfigures V4L2 + ISP for the new stream
     * set, (3) restarts the workers. */

    stopWorkers();

    /* Error-complete any requests still parked upstream of
     * PipelineThread's inFlight drain: ctxs in mRequestQueue,
     * ctxs mid-stage in RequestThread (it was unblocked out of
     * pushBlocking by the queue's requestStop), ctxs in
     * mPipelineQueue. Without this the framework's output buffers
     * stay dequeued on the HAL side across the reconfigure and
     * the next processCaptureRequest sequence times out with
     * "wait for output buffer return timed out after 3000ms". */
    errorCompletePendingRequests();

    /* Drop the StatsWorker's in-progress cycle before the ISP ring
     * gets resized out from under it. Without this reset the worker's
     * currentJob.bayer would still point into the old input ring after
     * ensureBuffers (below) unmaps it on a resolution change. */
    if (mStatsWorker) mStatsWorker->reset();

    ALOGD("V4L2 target resolution: %ux%u, soft_isp=%d",
          width, height, mSoftIspEnabled);

    if(!mDev->setStreaming(false)) {
        ALOGE("Could not stop streaming");
        return NO_INIT;
    }

    /* Allocate Vulkan input buffers at the target resolution before V4L2 so
     * we can hand their dma-buf fds to VIDIOC_REQBUFS(DMABUF). If any step of
     * the export fails we fall back to MMAP + memcpy silently. */
    mIsp->prewarm(width, height, mDev->pixelFormat());
    {
        int n = mIsp->inputBufferCount();
        if (n > 0 && n == V4L2DEVICE_BUF_COUNT) {
            int fds[V4L2DEVICE_BUF_COUNT];
            bool allOk = true;
            for (int i = 0; i < n; i++) {
                fds[i] = mIsp->exportInputBufferFd(i);
                if (fds[i] < 0) { allOk = false; break; }
            }
            if (allOk) {
                mDev->setDmaBufFds(fds, n);
                ALOGD("V4L2 input: DMABUF mode, %d slots", n);
            } else {
                for (int i = 0; i < n; i++) if (fds[i] >= 0) ::close(fds[i]);
                ALOGW("V4L2 input: DMABUF export failed, falling back to MMAP");
            }
        }
    }

    if(!mDev->setResolution(width, height)) {
        ALOGE("Could not set resolution");
        return NO_INIT;
    }

    ALOGV("+-------------------------------------------------------------------------------");
    ALOGV("| STREAMS AFTER CHANGES");
    ALOGV("+-------------------------------------------------------------------------------");
    for(size_t i = 0; i < streamList->num_streams; ++i) {
        const camera3_stream_t *newStream = streamList->streams[i];
        ALOGV("| p=%p  fmt=0x%.2x  type=%u  usage=0x%.8x  size=%4ux%-4u  buf_no=%u",
              newStream,
              newStream->format,
              newStream->stream_type,
              newStream->usage,
              newStream->width,
              newStream->height,
              newStream->max_buffers);
    }
    ALOGV("+-------------------------------------------------------------------------------");

    /* mExposure is long-lived (built in openDevice); push defaults
     * each reconfigure so sensor controls reflect the new session. */
    if (mExposure) mExposure->applyDefaults();

    if(!mDev->setStreaming(true)) {
        ALOGE("Could not start streaming");
        return NO_INIT;
    }

    startWorkers();
    return NO_ERROR;
}

int Camera::registerStreamBuffers(const camera3_stream_buffer_set_t *bufferSet) {
    DBGUTILS_AUTOLOGCALL(__func__);
    Mutex::Autolock lock(mMutex);
    ALOGV("+-------------------------------------------------------------------------------");
    ALOGV("| BUFFERS FOR STREAM %p", bufferSet->stream);
    ALOGV("+-------------------------------------------------------------------------------");
    for (size_t i = 0; i < bufferSet->num_buffers; ++i) {
        ALOGV("| p=%p", bufferSet->buffers[i]);
    }
    ALOGV("+-------------------------------------------------------------------------------");

    return OK;
}

int Camera::processCaptureRequest(camera3_capture_request_t *request) {
    assert(request != NULL);

    FPSCOUNTER_HERE(120);
    ALOGV("--- capture request --- f=%-5u in_buf=%p  out_bufs=%p[%u] --- fps %4.1f (avg %4.1f)",
          request->frame_number,
          request->input_buffer,
          request->output_buffers,
          request->num_output_buffers,
          FPSCOUNTER_VALUE(1), FPSCOUNTER_VALUE());

    /* Acknowledge unused input buffer (reprocess slot is reserved but
     * not consumed yet). */
    if (request->input_buffer) {
        request->input_buffer->release_fence = -1;
    }

    std::unique_ptr<PipelineContext> ctx(new PipelineContext());
    ctx->sequence            = request->frame_number;
    ctx->request.frameNumber = request->frame_number;
    ctx->tAccepted           = systemTime();

    {
        Mutex::Autolock lock(mMutex);

        if (!mRequestThread || !mRequestThread->isRunning()) {
            ALOGE("processCaptureRequest: request thread not running");
            notifyError(request->frame_number, NULL, CAMERA3_MSG_ERROR_REQUEST);
            return INVALID_OPERATION;
        }

        if (request->settings == NULL && mLastRequestSettings.isEmpty()) {
            ALOGE("First request does not have metadata");
            notifyError(request->frame_number, NULL, CAMERA3_MSG_ERROR_REQUEST);
            return BAD_VALUE;
        }

        if (request->settings) {
            ctx->request.settings = request->settings;
            /* Update the cache synchronously: the worker that would
             * otherwise write it runs far behind the binder thread,
             * so a request with settings=NULL landing before frame N's
             * worker finishes would otherwise see an empty cache. */
            mLastRequestSettings = request->settings;
        } else {
            ctx->request.settings = mLastRequestSettings;
        }
    }

    /* Deep-copy output buffer descriptors. Acquire-fence ownership
     * transfers to the HAL via UniqueFd. */
    ctx->request.outputBuffers.resize(request->num_output_buffers);
    for (uint32_t i = 0; i < request->num_output_buffers; ++i) {
        const camera3_stream_buffer &src = request->output_buffers[i];
        CaptureRequest::Buffer &dst = ctx->request.outputBuffers[i];
        dst.stream = src.stream;
        dst.buffer = src.buffer;
        if (src.acquire_fence >= 0) {
            dst.acquireFence.reset(src.acquire_fence);
        }
    }

    PipelineContext *raw = ctx.get();
    mTracker->add(std::move(ctx));

    if (!mRequestQueue->push(raw)) {
        /* Soft cap at REQUEST_QUEUE_CAPACITY (256) — this means the
         * framework has more requests outstanding than its own
         * stream.max_buffers contract allows. Reclaim and log-fatal. */
        std::unique_ptr<PipelineContext> reclaim = mTracker->removeBySequence(raw->sequence);
        (void)reclaim;
        LOG_ALWAYS_FATAL("RequestQueue overflow (framework exceeded stream.max_buffers)");
    }

    return NO_ERROR;
}

void Camera::buildInfrastructure() {
    /* Called under mMutex from the first openDevice. Builds all
     * long-lived per-camera state; lives until ~Camera. */
    if (mInfrastructureBuilt) return;

    mIsp = createIspPipeline();
    if (!mIsp || !mIsp->init()) {
        ALOGE("ISP init failed");
        if (mIsp) { delete mIsp; mIsp = NULL; }
        return;
    }
    mIsp->setEnabled(mSoftIspEnabled);

    /* Optical-black bias — set on the ISP only here; the same value is
     * also pushed into StatsWorker further down, once that object has
     * been constructed. Shared between the demosaic shader and the
     * CPU stats encoder so both read the same signal space; mismatched
     * bias on the stats side lets the offset dominate raw means on
     * dark scenes and collapses the R/G/B ratios gray-world AWB
     * needs. */
    if (mTuning.isLoaded())
        mIsp->setBlackLevel((uint32_t)mTuning.opticalBlack().r);

    /* WB prior + seed CCM. Takes the hottest-CCT CcmSet the tuning
     * ships as the sensor's calibrated "daylight" anchor — no
     * literal CCT appears here. BasicIpa stores the same prior as
     * its lastWbR / lastWbB so the very first frame (before any
     * stats arrive) renders through sensor-correct WB gains; the
     * mCcmQ10 buffer handed to setCcm is then rewritten every AWB
     * tick via BasicIpa's U → CCT → LERP path. Tunings without
     * awb.v4 fall back to nearest-CCT pick at the same daylight
     * anchor via the legacy ccmForCctQ10 route. */
    /* `wbGainPrior` now carries ready-to-apply shader gains —
     * (G/R, 1.0, G/B) of NVIDIA's FusionInitLight raw anchor, post
     * black-level. BasicIpa consumes wbGainPrior[0] / wbGainPrior[1]
     * as wbRPrior and [2] / [1] as wbBPrior (both yield G/{R,B} as
     * the gray-world R and B multipliers). For the CCM seed we need
     * the same U = ln(G/B) the per-frame AWB loop uses, which is
     * directly wbGainPrior[2]. */
    float wbGainPrior[3];
    mTuning.defaultWbGain(wbGainPrior);
    if (mTuning.awbParams().loaded && wbGainPrior[2] > 0.f) {
        const float uSeed = logf(wbGainPrior[2]);
        mTuning.ccmForCctLerpQ10(mTuning.estimateCctFromU(uSeed), mCcmQ10);
    } else if (!mTuning.ccmSets().empty()) {
        int daylightCctK = mTuning.ccmSets().front().cctK;
        for (const auto &s : mTuning.ccmSets())
            if (s.cctK > daylightCctK) daylightCctK = s.cctK;
        mTuning.ccmForCctQ10(daylightCctK, mCcmQ10);
    }
    mIsp->setCcm(mCcmQ10);

    /* IPA built before the AF controller so the latter can take a
     * non-null Ipa* — AF queries `isAeConverged()` to gate
     * continuous-mode retriggers and toggles `setAeLock` across
     * each sweep. The IPA reads tuning + isp + ccm only at
     * construction; nothing else needed yet at this point. */
    mIpa.reset(new BasicIpa(mSensorCfg, mIsp, &mTuning, wbGainPrior, mCcmQ10));

    /* Soft-ISP path owns exposure + AF; HW-ISP firmware owns them
     * otherwise. */
    if (mSoftIspEnabled) {
        mExposure = new ExposureControl(mDev, mSensorCfg);
        mAf       = new AutoFocusController(mDev, mIsp, mIpa.get(), &mTuning);
    }

    mJpeg = new JpegEncoder(mIsp);

    BufferProcessor::Deps bpDeps;
    bpDeps.isp  = mIsp;
    bpDeps.jpeg = mJpeg;
    mBufferProcessor = new BufferProcessor(bpDeps);

    mBayerSource.reset(new V4l2Source(mDev));
    mTracker.reset(new InFlightTracker());
    mRequestQueue.reset(new EventQueue<PipelineContext*>(REQUEST_QUEUE_CAPACITY));
    mPipelineQueue.reset(new EventQueue<PipelineContext*>(PIPELINE_QUEUE_CAPACITY));
    mRequestPipeline.reset(new Pipeline());
    mStatsWorker.reset(new StatsWorker());
    if (mTuning.isLoaded())
        mStatsWorker->setBlackLevel((uint32_t)mTuning.opticalBlack().r);

    /* DelayedControls plumbing for the AE / gain push from the IPA's
     * processStats() into the ApplySettingsStage write path. The
     * ring is seeded from the sensor's silicon control-delay config
     * so push / apply sequences align; built here so the request
     * pipeline can take the pointer at construction. */
    {
        DelayedControls::Config cfg;
        for (int i = 0; i < DelayedControls::COUNT; ++i) {
            cfg.delay[i]        = mSensorCfg.controlDelay[i];
            cfg.defaultValue[i] = 0;
        }
        mDelayedControls.reset(new DelayedControls(cfg));
    }

    /* RequestThread pipeline — runs Apply / Shutter / Capture on the
     * binder-adjacent thread. DemosaicBlit and ResultDispatch live on
     * PipelineThread behind the fence-fd poll set, so they are owned
     * standalone rather than appended here. */
    {
        ApplySettingsStage::Deps d;
        d.exposure        = mExposure;
        d.af              = mAf;
        d.sensorCfg       = &mSensorCfg;
        d.delayedControls = mDelayedControls.get();
        mRequestPipeline->appendStage(
            std::unique_ptr<PipelineStage>(new ApplySettingsStage(d)));
    }

    mRequestPipeline->appendStage(
        std::unique_ptr<PipelineStage>(new ShutterNotifyStage(&mCallbackOps)));

    {
        CaptureStage::Deps d;
        d.bayerSource = mBayerSource.get();
        d.isp         = mIsp;
        d.af          = mAf;
        mRequestPipeline->appendStage(
            std::unique_ptr<PipelineStage>(new CaptureStage(d)));
    }

    {
        StatsDispatchStage::Deps d;
        d.isp         = mIsp;
        d.statsWorker = mStatsWorker.get();
        d.bayerSource = mBayerSource.get();
        mRequestPipeline->appendStage(
            std::unique_ptr<PipelineStage>(new StatsDispatchStage(d)));
    }

    {
        DemosaicBlitStage::Deps d;
        d.bufferProcessor = mBufferProcessor;
        d.bayerSource     = mBayerSource.get();
        d.jpegBufferSize  = &mJpegBufferSize;
        mDemosaicBlitStage.reset(new DemosaicBlitStage(d));
    }

    {
        ResultDispatchStage::Deps d;
        d.callbackOps = &mCallbackOps;
        d.bayerSource = mBayerSource.get();
        d.af          = mAf;
        d.sensorCfg   = &mSensorCfg;
        mResultDispatchStage.reset(new ResultDispatchStage(d));
    }

    {
        StatsProcessStage::Deps d;
        d.ipa             = mIpa.get();
        d.delayedControls = mDelayedControls.get();
        d.sensorCfg       = &mSensorCfg;
        d.statsWorker     = mStatsWorker.get();
        d.af              = mAf;
        mStatsProcessStage.reset(new StatsProcessStage(d));
    }

    mRequestThread.reset(new RequestThread(mRequestQueue.get(),
                                           mRequestPipeline.get(),
                                           mPipelineQueue.get(),
                                           mTracker.get()));
    {
        PipelineThread::Deps d;
        d.queue          = mPipelineQueue.get();
        d.demosaicBlit   = mDemosaicBlitStage.get();
        d.statsProcess   = mStatsProcessStage.get();
        d.resultDispatch = mResultDispatchStage.get();
        d.bayerSource    = mBayerSource.get();
        d.tracker        = mTracker.get();
        d.maxInFlight    = PIPELINE_MAX_IN_FLIGHT;
        mPipelineThread.reset(new PipelineThread(d));
    }

    mInfrastructureBuilt = true;
}

void Camera::destroyInfrastructure() {
    mPipelineThread.reset();
    mRequestThread.reset();
    mResultDispatchStage.reset();
    mStatsProcessStage.reset();
    mDemosaicBlitStage.reset();
    mRequestPipeline.reset();
    mStatsWorker.reset();
    mDelayedControls.reset();
    mIpa.reset();
    mTracker.reset();
    mPipelineQueue.reset();
    mRequestQueue.reset();
    mBayerSource.reset();

    delete mBufferProcessor; mBufferProcessor = NULL;
    delete mAf;              mAf = NULL;
    delete mExposure;        mExposure = NULL;
    delete mJpeg;            mJpeg = NULL;
    if (mIsp) { mIsp->destroy(); delete mIsp; mIsp = NULL; }

    mInfrastructureBuilt = false;
}

void Camera::errorCompletePendingRequests() {
    if (!mTracker || !mResultDispatchStage) return;
    auto pending = mTracker->drainAll();
    for (auto &ctx : pending) {
        if (!ctx) continue;
        ctx->errorCode = CAMERA3_MSG_ERROR_REQUEST;
        mResultDispatchStage->process(*ctx);
    }
}

void Camera::stopWorkers() {
    /* Ordering:
     * 1. Bayer: parked CaptureStage waits on acquireNextFrame cv —
     *    stop() notifies it with stopping=true so the stage unwinds.
     * 2. pipelineQueue.requestStop: RequestThread may be blocked
     *    inside pushBlocking; flipping the queue's stopping flag wakes
     *    it with a false return so it can exit.
     * 3. RequestThread join: stopFd signal + stopFlag for the inner
     *    loop, then join. The ctx it was holding (if any) stays in
     *    the tracker for closeDevice's drainAll.
     * 4. PipelineThread join: stopFd wakes the poll; threadLoop's
     *    tail drains all remaining in-flight contexts — blocking
     *    WaitForFences with a per-fence timeout — and runs
     *    ResultDispatch on each so framework buffers return cleanly.
     *
     * GPU drain now lives inside PipelineThread's stop path — no more
     * waitForPreviousFrame at this layer. */
    if (mBayerSource)    mBayerSource->stop();
    if (mPipelineQueue)  mPipelineQueue->requestStop();
    if (mRequestThread)  mRequestThread->stop();
    /* StatsWorker is stopped after RequestThread (so no more submits
     * arrive) but can run alongside PipelineThread's drain — the
     * consumer StatsProcessStage just peeks a stale snapshot once the
     * worker is quiesced. */
    if (mStatsWorker)    mStatsWorker->stop();
    if (mPipelineThread) mPipelineThread->stop();
}

void Camera::startWorkers() {
    /* Clear the stopping flag that stopWorkers flipped on the pipeline
     * queue; otherwise pushBlocking in the new session would fail
     * instantly. */
    if (mPipelineQueue) mPipelineQueue->clearStop();

    if (mBayerSource && !mBayerSource->isRunning()) {
        if (!mBayerSource->start()) {
            ALOGE("BayerSource start failed");
            return;
        }
    }
    /* StatsWorker before RequestThread: the producer side must have a
     * live sink to post Bayer jobs into from the very first
     * StatsDispatchStage run. */
    if (mStatsWorker && !mStatsWorker->isRunning()) {
        if (!mStatsWorker->start("CamStats")) {
            ALOGE("StatsWorker start failed, unwinding BayerSource");
            if (mBayerSource) mBayerSource->stop();
            return;
        }
    }
    /* PipelineThread before RequestThread: the downstream consumer
     * must be live before RequestThread starts pushing to pipelineQueue.
     * Swapping the order would have RequestThread block on pushBlocking
     * forever. */
    if (mPipelineThread && !mPipelineThread->isRunning()) {
        if (!mPipelineThread->start("CamPipeline")) {
            ALOGE("PipelineThread start failed, unwinding StatsWorker + BayerSource");
            if (mStatsWorker) mStatsWorker->stop();
            if (mBayerSource) mBayerSource->stop();
            return;
        }
    }
    if (mRequestThread && !mRequestThread->isRunning()) {
        if (!mRequestThread->start("CamRequest")) {
            ALOGE("RequestThread start failed, unwinding PipelineThread + StatsWorker + BayerSource");
            if (mPipelineThread) mPipelineThread->stop();
            if (mStatsWorker)    mStatsWorker->stop();
            if (mBayerSource)    mBayerSource->stop();
            return;
        }
    }
}

inline void Camera::notifyShutter(uint32_t frameNumber, uint64_t timestamp) {
    camera3_notify_msg_t msg;
    msg.type = CAMERA3_MSG_SHUTTER;
    msg.message.shutter.frame_number = frameNumber;
    msg.message.shutter.timestamp = timestamp;
    mCallbackOps->notify(mCallbackOps, &msg);
}

void Camera::notifyError(uint32_t frameNumber, camera3_stream_t *stream, int errorCode) {
    camera3_notify_msg_t msg;
    msg.type = CAMERA3_MSG_ERROR;
    msg.message.error.frame_number = frameNumber;
    msg.message.error.error_stream = stream;
    msg.message.error.error_code   = errorCode;
    mCallbackOps->notify(mCallbackOps, &msg);
}

void Camera::processCaptureResult(uint32_t frameNumber, const camera_metadata_t *result, const Vector<camera3_stream_buffer> &buffers) {
    camera3_capture_result captureResult;
    captureResult.frame_number = frameNumber;
    captureResult.result = result;
    captureResult.num_output_buffers = buffers.size();
    captureResult.output_buffers = buffers.array();
    captureResult.input_buffer = NULL;
    captureResult.partial_result = 0;

    mCallbackOps->process_capture_result(mCallbackOps, &captureResult);
}

/******************************************************************************\
                                STATIC WRAPPERS
\******************************************************************************/

int Camera::sClose(hw_device_t *device) {
    /* TODO: check device module */
    Camera *thiz = static_cast<Camera *>(reinterpret_cast<camera3_device_t *>(device));
    return thiz->closeDevice();
}

int Camera::sInitialize(const camera3_device *device, const camera3_callback_ops_t *callback_ops) {
    /* TODO: check pointers */
    Camera *thiz = static_cast<Camera *>(const_cast<camera3_device *>(device));
    return thiz->initialize(callback_ops);
}

int Camera::sConfigureStreams(const camera3_device *device, camera3_stream_configuration_t *stream_list) {
    /* TODO: check pointers */
    Camera *thiz = static_cast<Camera *>(const_cast<camera3_device *>(device));
    return thiz->configureStreams(stream_list);
}

int Camera::sRegisterStreamBuffers(const camera3_device *device, const camera3_stream_buffer_set_t *buffer_set) {
    /* TODO: check pointers */
    Camera *thiz = static_cast<Camera *>(const_cast<camera3_device *>(device));
    return thiz->registerStreamBuffers(buffer_set);
}

const camera_metadata_t * Camera::sConstructDefaultRequestSettings(const camera3_device *device, int type) {
    /* TODO: check pointers */
    Camera *thiz = static_cast<Camera *>(const_cast<camera3_device *>(device));
    return thiz->constructDefaultRequestSettings(type);
}

int Camera::sProcessCaptureRequest(const camera3_device *device, camera3_capture_request_t *request) {
    /* TODO: check pointers */
    Camera *thiz = static_cast<Camera *>(const_cast<camera3_device *>(device));
    return thiz->processCaptureRequest(request);
}

void Camera::sGetMetadataVendorTagOps(const camera3_device *device, vendor_tag_query_ops_t *ops) {
    /* TODO: implement */
    ALOGD("%s: IMPLEMENT ME!", __FUNCTION__);
}

void Camera::sDump(const camera3_device *device, int fd) {
    /* TODO: implement */
    ALOGD("%s: IMPLEMENT ME!", __FUNCTION__);
}

int Camera::sFlush(const camera3_device *device) {
    return OK;
}

camera3_device_ops_t Camera::sOps = {
    .initialize                         = Camera::sInitialize,
    .configure_streams                  = Camera::sConfigureStreams,
    .register_stream_buffers            = Camera::sRegisterStreamBuffers,
    .construct_default_request_settings = Camera::sConstructDefaultRequestSettings,
    .process_capture_request            = Camera::sProcessCaptureRequest,
    .get_metadata_vendor_tag_ops        = Camera::sGetMetadataVendorTagOps,
    .dump                               = Camera::sDump,
    .flush                              = Camera::sFlush,
    .reserved = {0}
};

}; /* namespace android */
