/*
 * Copyright (C) 2026 Artem Bambalov
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
#include "CameraMetadataCompat.h"
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
#include "pipeline/CaptureRequest.h"
#include "pipeline/Pipeline.h"
#include "pipeline/DeviceErrorReporter.h"
#include "pipeline/stages/ApplySettingsStage.h"
#include "pipeline/stages/ShutterNotifyStage.h"
#include "pipeline/stages/DemosaicBlitStage.h"
#include "pipeline/stages/ResultDispatchStage.h"
#include "pipeline/stages/StatsProcessStage.h"
#include "ipa/Ipa3A.h"
#include "CaptureEngine.h"
#include "FinishEngine.h"
#include "Frame.h"
#include "FrameRegistry.h"
#include "IspEngine.h"
#include "JpegEngine.h"
#include "ResultEngine.h"
#include "StatsEngine.h"

extern camera_module_t HAL_MODULE_INFO_SYM;

namespace {
/* Frames the HAL holds at once before a new request is failed on
 * arrival. The framework keeps at most max_buffers (StreamConfig: 4)
 * of each stream outstanding, and StreamConfig takes three streams at
 * most, so 12 is what a framework keeping its contract can reach; the
 * rest is headroom. */
constexpr size_t kFrameCeiling = 16;

/* How long stopping waits for the frames past the GPU (JPEG encodes,
 * results) to go out normally before the rest go out in error. */
constexpr int kStopDrainTimeoutMs = 1000;
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
    , mFacing(facing)
    , mCallbackOps(NULL)
    , mJpegBufferSize(0)
    , mSoftIspEnabled(true)
    , mInfrastructureBuilt(false) {
    DBGUTILS_AUTOLOGCALL(__func__);
    for(size_t i = 0; i < NELEM(mDefaultRequestSettings); i++) {
        mDefaultRequestSettings[i] = NULL;
    }

    common.tag      = HARDWARE_DEVICE_TAG;
    common.version  = CAMERA_DEVICE_API_VERSION_3_4;
    common.module   = &HAL_MODULE_INFO_SYM.common;
    common.close    = Camera::sClose;
    ops             = &sOps;
    priv            = NULL;

    mValid = true;
    mIsp = NULL;
    mAf = NULL;
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
    /* Without a tuning, the same stand-in the static metadata used to
     * fall back on. */
    mSensorCfg.focalLengthMm = mTuning.isLoaded()
        ? mTuning.module().focalLengthMm : 3.30f;

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
    stopEngines();
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
    /* Source of truth is the per-module tuning (`module.sensor_orientation_degrees`).
     * Falls to 0 if the tuning didn't load — wrong but not crashing,
     * and "tuning didn't load" already means the HAL is degraded. */
    info->orientation = mTuning.module().sensorOrientationDegrees;
    info->device_version = CAMERA_DEVICE_API_VERSION_3_4;
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
        mSensorCfg.gainMin     = gMin;
        mSensorCfg.gainMax     = gMax;
        mSensorCfg.gainDefault = gDef;
        ALOGD("Gain: min=%d max=%d def=%d", gMin, gMax, gDef);
    }
    int32_t eMin, eMax, eDef;
    if (mDev->queryControl(V4L2_CID_EXPOSURE, &eMin, &eMax, &eDef)) {
        mSensorCfg.exposureMin     = eMin;
        mSensorCfg.exposureMax     = eMax;
        mSensorCfg.exposureDefault = eDef;
        ALOGD("Exposure: min=%d max=%d def=%d (us)", eMin, eMax, eDef);
    }
}

int Camera::openDevice(hw_device_t **device) {
    DBGUTILS_AUTOLOGCALL(__func__);
    Mutex::Autolock lock(mMutex);
    mDev->connect();
    *device = &common;

    /* Open focuser if the tuning declares this sensor has AF. The
     * actual lens subdev is discovered via the media controller —
     * no paths or hardware ids in user config. */
    ALOGI("openDevice: facing=%d hasAf=%d", mFacing, (int)mTuning.hasAf());
    if (mTuning.hasAf()) {
        if (!mDev->openFocuser())
            ALOGW("openDevice: focuser open failed; AF will be inactive");
    }

    /* Pull driver-advertised ranges into mSensorCfg before anything
     * consumes it. Ipa3A's ctor (inside buildInfrastructure) uses
     * gainDefault / exposureDefault / gainMax / frameLenDefault to
     * seed AE state — querying afterwards leaves Ipa3A holding
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

    /* Stop the engines before taking mMutex: one may still be handing
     * a result back through a framework callback. Every frame held goes
     * back to the framework before this returns. */
    stopEngines();

    Mutex::Autolock lock(mMutex);

    /* Drop per-session state so the next open starts from a clean
     * slate. Infrastructure (ISP core, 3A, BufferProcessor, pipeline,
     * worker threads) survives and will be reused. */
    mLastRequestSettings.clear();
    mLastConfigWidth  = 0;
    mLastConfigHeight = 0;
    mLastAeMode       = 0xff;
    if (mAf)              mAf->reset();
    if (mApplySettingsStage) mApplySettingsStage->reset();
    if (mDeviceError)     mDeviceError->rearm();
    if (mIsp)             mIsp->onSessionClose();
    if (mIpa)             mIpa->reset();
    if (mDelayedControls) mDelayedControls->reset();
    if (mStatsEngine)     mStatsEngine->reset();

    mDev->disconnect();
    return NO_ERROR;
}

camera_metadata_t *Camera::staticCharacteristics() {
    if(mStaticCharacteristics)
        return mStaticCharacteristics;
    mStaticCharacteristics = CameraStaticMetadata::build(mDev, mFacing, &mTuning,
                                                          mSensorCfg,
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

    /* camera3.h: an unknown template type (vendor ones included) gets
     * NULL. mDefaultRequestSettings is indexed by type. */
    if (type < CAMERA3_TEMPLATE_PREVIEW || type >= CAMERA3_TEMPLATE_COUNT) {
        ALOGE("constructDefaultRequestSettings: unknown template %d", type);
        return NULL;
    }

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

    /* Skip the V4L2/Vulkan reinit when only the output stream set has
     * changed without affecting the capture resolution. Apps like Open
     * Camera reissue configure_streams on AE_MODE auto↔manual toggles
     * even when nothing about the streams really changed; the full
     * stopEngines + STREAMOFF + prewarm + STREAMON + startEngines cycle
     * costs ~3 s on Tegra K1 and surfaces as the camera "reopening"
     * between mode switches. New output streams build their gralloc
     * cache entries lazily on first blit, so no per-stream setup is
     * needed here. */
    if (mLastConfigWidth == width && mLastConfigHeight == height) {
        ALOGD("configureStreams: same %ux%u capture as last — no-op reinit",
              width, height);
        return NO_ERROR;
    }
    mLastConfigWidth  = width;
    mLastConfigHeight = height;

    /* Infrastructure (ISP, 3A, BufferProcessor, BayerSource, pipeline,
     * request thread) was built on the first openDevice and lives for
     * the Camera instance's lifetime. This function only (1) stops
     * the workers, (2) reconfigures V4L2 + ISP for the new stream
     * set, (3) restarts the workers. */

    /* Every frame held goes back to the framework before the capture
     * is torn down; requests come again once the streams are set. */
    stopEngines();

    /* Drop the stats cycle in progress before the ISP ring gets resized
     * out from under it. */
    if (mStatsEngine) mStatsEngine->reset();

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

    /* Push exposure / gain defaults so the sensor has a sane baseline
     * before STREAMON. The driver-queried defaults from
     * populateSensorConfigFromDriver are the "what the sensor itself
     * boots to" values; apply them once per reconfigure so a session
     * starting from a previous frame's leftover state lands on the
     * documented default. AE / IPA take over from frame 1 onward. */
    if (mDev) {
        mDev->setControl(V4L2_CID_EXPOSURE, mSensorCfg.exposureDefault);
        mDev->setControl(V4L2_CID_GAIN,     mSensorCfg.gainDefault);
    }

    if(!mDev->setStreaming(true)) {
        ALOGE("Could not start streaming");
        return NO_INIT;
    }

    if (!startEngines()) {
        ALOGE("configureStreams: the engines did not start");
        return NO_INIT;
    }
    return NO_ERROR;
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

    auto ctx = std::make_shared<mocha::camera::Frame>();
    ctx->frameNumber         = request->frame_number;
    ctx->request.frameNumber = request->frame_number;
    ctx->tAccepted           = systemTime();

    {
        Mutex::Autolock lock(mMutex);

        if (!mEnginesRunning) {
            ALOGE("processCaptureRequest: the engines are not running");
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

        ctx->stillCapture = isStillCapture(*request, ctx->request.settings);

        /* Open Camera and similar Camera2 apps don't issue an explicit
         * flush() on AE_MODE manual→auto toggles, so the framework
         * leaves whatever long manual exposure was in flight to drain
         * naturally — observable as "manual→auto takes seconds" in
         * preview. Detect the transition here and give up the frames
         * still waiting for their Bayer, through the same cancellation
         * flush() uses. A frame that already has its Bayer is past the
         * wait and completes; a snapshot is never given up. Auto→manual
         * has nothing long to drain. The new request itself proceeds
         * normally past this point. */
        uint8_t curAeMode = ANDROID_CONTROL_AE_MODE_ON;
        if (ctx->request.settings.exists(ANDROID_CONTROL_AE_MODE)) {
            curAeMode = *ctx->request.settings.find(ANDROID_CONTROL_AE_MODE).data.u8;
        }
        if (mLastAeMode == ANDROID_CONTROL_AE_MODE_OFF &&
            curAeMode != ANDROID_CONTROL_AE_MODE_OFF) {
            ALOGD("processCaptureRequest: AE_MODE %u→%u, cancelling frames waiting for Bayer",
                  (unsigned)mLastAeMode, (unsigned)curAeMode);
            cancelInFlight([](const mocha::camera::Frame &c) {
                return !c.stillCapture &&
                       !c.hasBayer.load(std::memory_order_acquire);
            });
        }
        mLastAeMode = curAeMode;
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

    /* The frame's first token, taken before the result engine sees
     * the frame: it waits for the frame until the last token goes. */
    mocha::camera::FrameToken token(ctx);
    ctx->settled = mResultEngine->settledWake();

    if (!mRegistry->add(ctx)) {
        /* More frames held than a framework keeping its stream
         * contract can send. Camera3 gives no return code for that;
         * the request goes out at once as a failed one, in order. */
        ALOGE("frame %u: over the ceiling of %zu frames held — failed",
              ctx->frameNumber, kFrameCeiling);
        ctx->status.fail(NO_MEMORY);
    }
    /* A request arriving while flush() waits is cancelled like the
     * ones in flight. flush sets mFlushing before cancelling the
     * registry, so a frame added after that cancel sees the flag. */
    if (mFlushing.load()) ctx->status.cancel();

    /* Results go out in the order frames reach the result engine:
     * the order requests arrive in. */
    if (!mResultEngine->submit(ctx)) {
        ALOGE("frame %u: the result engine is stopped", ctx->frameNumber);
        mRegistry->remove(ctx->frameNumber);
        return -ENODEV;
    }
    /* A capture engine that refuses lets the token go; the frame goes
     * out as a failed request. */
    mCaptureEngine->submit(std::move(token));

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
     * also pushed into the stats engine further down, once that object has
     * been constructed. Shared between the demosaic shader and the
     * CPU stats encoder so both read the same signal space; mismatched
     * bias on the stats side lets the offset dominate raw means on
     * dark scenes and collapses the R/G/B ratios gray-world AWB
     * needs. */
    if (mTuning.isLoaded())
        mIsp->setBlackLevel((uint32_t)mTuning.opticalBlack().r);

    /* WB prior + seed CCM. Takes the hottest-CCT CcmSet the tuning
     * ships as the sensor's calibrated "daylight" anchor — no
     * literal CCT appears here. Ipa3A stores the same prior as
     * its lastWbR / lastWbB so the very first frame (before any
     * stats arrive) renders through sensor-correct WB gains; the
     * mCcmQ10 buffer handed to setCcm is then rewritten every AWB
     * tick via Ipa3A's U → CCT → LERP path. Tunings without
     * awb.v4 fall back to nearest-CCT pick at the same daylight
     * anchor via the legacy ccmForCctQ10 route. */
    /* `wbGainPrior` now carries ready-to-apply shader gains —
     * (G/R, 1.0, G/B) of NVIDIA's FusionInitLight raw anchor, post
     * black-level. Ipa3A consumes wbGainPrior[0] / wbGainPrior[1]
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

    /* AF controller built before IPA so the IPA can hold a non-null
     * AF pointer for in-coordinator dispatch (af->process is called
     * from Ipa3A::processStats and its result drives the AE / AWB
     * lock toggles via the coordinator). */
    if (mSoftIspEnabled) {
        const Resolution sensorRes = mDev->sensorResolution();
        mAf = new AutoFocusController(&mTuning, (int)sensorRes.width,
                                      (int)sensorRes.height);
    }

    {
        Ipa3A *ipa = new Ipa3A(mSensorCfg, mIsp, mAf, &mTuning,
                               wbGainPrior, mCcmQ10);
        /* The one place the lens is moved from: AF decides, the IPA
         * coordinator writes, on the IPA's own thread. */
        V4l2Device *dev = mDev;
        ipa->setLensWriter([dev](int32_t pos) { dev->setFocusPosition(pos); });
        mIpa.reset(ipa);
    }

    mJpeg = new JpegEncoder();

    BufferProcessor::Deps bpDeps;
    bpDeps.isp            = mIsp;
    bpDeps.jpeg           = mJpeg;
    bpDeps.jpegBufferSize = &mJpegBufferSize;
    mBufferProcessor = new BufferProcessor(bpDeps);

    mPartialEmitter.reset(new CameraCallbackEmitter(&mCallbackOps));
    mBayerSource.reset(new V4l2Source(mDev));
    mRegistry.reset(new mocha::camera::FrameRegistry(kFrameCeiling));
    mDeviceError.reset(new DeviceErrorReporter(&mCallbackOps));
    mStatsEngine.reset(new mocha::camera::StatsEngine());
    if (mTuning.isLoaded())
        mStatsEngine->setBlackLevel((uint32_t)mTuning.opticalBlack().r);

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

    /* The request's settings and its shutter, run by the capture
     * engine as each frame is captured. */
    mSettingsPipeline.reset(new Pipeline());
    {
        ApplySettingsStage::Deps d;
        d.dev             = mDev;
        d.af              = mAf;
        d.sensorCfg       = &mSensorCfg;
        d.delayedControls = mDelayedControls.get();
        mApplySettingsStage = new ApplySettingsStage(d);
        mSettingsPipeline->appendStage(
            std::unique_ptr<PipelineStage>(mApplySettingsStage));
    }
    mSettingsPipeline->appendStage(
        std::unique_ptr<PipelineStage>(new ShutterNotifyStage(&mCallbackOps)));

    {
        DemosaicBlitStage::Deps d;
        d.bufferProcessor = mBufferProcessor;
        d.bayerSource     = mBayerSource.get();
        d.isp             = mIsp;
        d.deviceError     = mDeviceError.get();
        mDemosaicBlitStage.reset(new DemosaicBlitStage(d));
    }

    {
        ResultDispatchStage::Deps d;
        d.callbackOps     = &mCallbackOps;
        d.emitter         = mPartialEmitter.get();
        d.af              = mAf;
        d.sensorCfg       = &mSensorCfg;
        d.delayedControls = mDelayedControls.get();
        mResultDispatchStage.reset(new ResultDispatchStage(d));
    }

    {
        StatsProcessStage::Deps d;
        d.ipa             = mIpa.get();
        d.delayedControls = mDelayedControls.get();
        d.sensorCfg       = &mSensorCfg;
        d.stats           = mStatsEngine.get();
        d.af              = mAf;
        d.emitter         = mPartialEmitter.get();
        mStatsProcessStage.reset(new StatsProcessStage(d));
    }

    /* The engines, downstream first: each takes the one after it. */
    {
        mocha::camera::ResultEngine::Deps d;
        d.dispatch = mResultDispatchStage.get();
        d.registry = mRegistry.get();
        mResultEngine.reset(new mocha::camera::ResultEngine(d));
    }
    mJpegEngine.reset(new mocha::camera::JpegEngine());
    {
        mocha::camera::FinishEngine::Deps d;
        d.bufferProcessor = mBufferProcessor;
        d.statsProcess    = mStatsProcessStage.get();
        d.jpeg            = mJpegEngine.get();
        mFinishEngine.reset(new mocha::camera::FinishEngine(d));
    }
    {
        mocha::camera::IspEngine::Deps d;
        d.demosaic    = mDemosaicBlitStage.get();
        d.finish      = mFinishEngine.get();
        d.deviceError = mDeviceError.get();
        mIspEngine.reset(new mocha::camera::IspEngine(d));
    }
    {
        mocha::camera::CaptureEngine::Deps d;
        d.source      = mBayerSource.get();
        d.isp         = mIsp;
        d.af          = mAf;
        d.settings    = mSettingsPipeline.get();
        d.demosaic    = mIspEngine.get();
        d.stats       = mStatsEngine.get();
        d.deviceError = mDeviceError.get();
        mCaptureEngine.reset(new mocha::camera::CaptureEngine(d));
    }

    mInfrastructureBuilt = true;
}

void Camera::destroyInfrastructure() {
    mCaptureEngine.reset();
    mIspEngine.reset();
    mFinishEngine.reset();
    mJpegEngine.reset();
    mResultEngine.reset();
    mStatsProcessStage.reset();
    mResultDispatchStage.reset();
    mDemosaicBlitStage.reset();
    mApplySettingsStage = nullptr;
    mSettingsPipeline.reset();
    mStatsEngine.reset();
    mDelayedControls.reset();
    mIpa.reset();
    mRegistry.reset();
    mDeviceError.reset();
    mBayerSource.reset();

    delete mBufferProcessor; mBufferProcessor = NULL;
    delete mAf;              mAf = NULL;
    delete mJpeg;            mJpeg = NULL;
    if (mIsp) { mIsp->destroy(); delete mIsp; mIsp = NULL; }

    mInfrastructureBuilt = false;
}

void Camera::cancelInFlight(
        const std::function<bool(const mocha::camera::Frame &)> &which) {
    if (mRegistry)      mRegistry->cancelIf(which);
    if (mCaptureEngine) mCaptureEngine->wake();
}

bool Camera::isStillCapture(const camera3_capture_request_t &request,
                            const CameraMetadata &settings) {
    for (uint32_t i = 0; i < request.num_output_buffers; ++i) {
        const camera3_stream_t *s = request.output_buffers[i].stream;
        if (s && s->format == HAL_PIXEL_FORMAT_BLOB) return true;
    }
    if (!settings.exists(ANDROID_CONTROL_CAPTURE_INTENT)) return false;
    switch (*settings.find(ANDROID_CONTROL_CAPTURE_INTENT).data.u8) {
        case ANDROID_CONTROL_CAPTURE_INTENT_STILL_CAPTURE:
        case ANDROID_CONTROL_CAPTURE_INTENT_VIDEO_SNAPSHOT:
        case ANDROID_CONTROL_CAPTURE_INTENT_ZERO_SHUTTER_LAG:
            return true;
        default:
            return false;
    }
}

void Camera::stopEngines() {
    if (!mEnginesRunning) return;
    mEnginesRunning = false;

    /* From the source towards the result. A stopped engine lets the
     * jobs it holds go, and with them their frames' tokens: those
     * frames go on to the result engine and out as failed requests.
     * The demosaic engine first gives a frame on the GPU its fence's
     * time. */
    mCaptureEngine->stop();
    mStatsEngine->stop();
    mIspEngine->stop();
    mFinishEngine->stop();

    /* Frames past the GPU -- committed, waiting on a JPEG or on their
     * turn -- go out normally if they can. */
    if (!mRegistry->waitEmpty(kStopDrainTimeoutMs))
        ALOGE("stopEngines: %zu frames still held after %d ms",
              mRegistry->count(), kStopDrainTimeoutMs);
    mJpegEngine->stop();
    mResultEngine->stop();

    /* Whatever is left goes out in error, from here; no engine runs. */
    for (const mocha::camera::FrameRef &frame : mRegistry->takeAll())
        mResultEngine->dispatchFailed(*frame);

    mBayerSource->stop();
}

bool Camera::startEngines() {
    if (mEnginesRunning) return true;
    if (!mInfrastructureBuilt) return false;

    /* Downstream first, so no engine hands work to one not yet
     * running; the source before the capture engine that drives it. */
    const bool started = mBayerSource->start()
                      && mResultEngine->start()
                      && mJpegEngine->start()
                      && mFinishEngine->start()
                      && mIspEngine->start()
                      && mStatsEngine->start()
                      && mCaptureEngine->start();
    if (!started) {
        ALOGE("startEngines: an engine did not start; stopping the rest");
        mEnginesRunning = true;
        stopEngines();
        return false;
    }
    mEnginesRunning = true;
    return true;
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

/******************************************************************************\
                                STATIC WRAPPERS
\******************************************************************************/

int Camera::sClose(hw_device_t *device) {
    if (!device) {
        ALOGE("sClose: NULL device");
        return -ENODEV;
    }
    Camera *thiz = static_cast<Camera *>(reinterpret_cast<camera3_device_t *>(device));
    return thiz->closeDevice();
}

int Camera::sInitialize(const camera3_device *device, const camera3_callback_ops_t *callback_ops) {
    if (!device) {
        ALOGE("sInitialize: NULL device");
        return -ENODEV;
    }
    Camera *thiz = static_cast<Camera *>(const_cast<camera3_device *>(device));
    return thiz->initialize(callback_ops);
}

int Camera::sConfigureStreams(const camera3_device *device, camera3_stream_configuration_t *stream_list) {
    if (!device) {
        ALOGE("sConfigureStreams: NULL device");
        return -ENODEV;
    }
    Camera *thiz = static_cast<Camera *>(const_cast<camera3_device *>(device));
    return thiz->configureStreams(stream_list);
}

const camera_metadata_t * Camera::sConstructDefaultRequestSettings(const camera3_device *device, int type) {
    if (!device) {
        ALOGE("sConstructDefaultRequestSettings: NULL device");
        return nullptr;
    }
    Camera *thiz = static_cast<Camera *>(const_cast<camera3_device *>(device));
    return thiz->constructDefaultRequestSettings(type);
}

int Camera::sProcessCaptureRequest(const camera3_device *device, camera3_capture_request_t *request) {
    if (!device) {
        ALOGE("sProcessCaptureRequest: NULL device");
        return -ENODEV;
    }
    Camera *thiz = static_cast<Camera *>(const_cast<camera3_device *>(device));
    return thiz->processCaptureRequest(request);
}

void Camera::sDump(const camera3_device *device, int fd) {
    /* TODO: implement */
    ALOGD("%s: IMPLEMENT ME!", __FUNCTION__);
}

int Camera::flush() {
    DBGUTILS_AUTOLOGCALL(__func__);

    /* camera3.h: return only once the HAL holds no more requests or
     * buffers, and as soon as possible. The HAL cancels the requests
     * in flight and lets the engines drain them; tearing the pipeline
     * down would itself cost the seconds we're trying to save.
     *
     * cancelInFlight cancels every frame held (one that already failed
     * or was committed keeps its outcome); each engine looks at the
     * status before its work and lets a cancelled frame go, and the
     * result engine sends it out as notify(ERROR_REQUEST) with the
     * framework's output buffers in error. Committed frames, too late
     * to cancel, complete normally. The capture engine is woken, so a
     * frame waiting on a long manual exposure is let go now rather
     * than a frame period later.
     *
     * The wait runs without mMutex: process_capture_request may be
     * called while flush is in progress and must return quickly. A
     * request arriving then is cancelled like the ones in flight
     * (mFlushing, see processCaptureRequest).
     *
     * The in-flight V4L2 capture finishes naturally; we don't
     * STREAMOFF here. The next process_capture_request after flush
     * acquires the following bayer normally. */
    mocha::camera::FrameRegistry *registry = nullptr;
    {
        Mutex::Autolock lock(mMutex);
        if (!mRegistry) return OK;
        registry = mRegistry.get();
        ALOGD("flush: inFlight=%zu", registry->count());
        mFlushing.store(true);
        cancelInFlight([](const mocha::camera::Frame &) { return true; });
    }

    const bool drained = registry->waitEmpty(kFlushTimeoutMs);
    mFlushing.store(false);
    if (!drained) {
        ALOGE("flush: %zu requests still in flight after %d ms",
              registry->count(), kFlushTimeoutMs);
    }
    return OK;
}

int Camera::sFlush(const camera3_device *device) {
    /* Camera is the camera3_device, like every other entry point here;
     * priv is never set (it was read here, so flush always failed with
     * -ENODEV). A flush after close finds no tracker and returns at
     * once. */
    if (!device) {
        ALOGE("sFlush: NULL device");
        return -ENODEV;
    }
    Camera *thiz = static_cast<Camera *>(const_cast<camera3_device *>(device));
    return thiz->flush();
}

camera3_device_ops_t Camera::sOps = {
    .initialize                         = Camera::sInitialize,
    .configure_streams                  = Camera::sConfigureStreams,
    /* Deprecated since HAL3.2 — the framework now hands gralloc
     * buffers to process_capture_request directly without a
     * pre-registration phase. Setting these non-NULL on a 3.2+
     * device causes Camera3Stream::finishConfiguration to bail
     * with -ENOSYS, which surfaces as
     * "configureStreamsLocked: Can't finish configuring output
     * stream" and leaves the session unable to start preview. */
    .register_stream_buffers            = NULL,
    .construct_default_request_settings = Camera::sConstructDefaultRequestSettings,
    .process_capture_request            = Camera::sProcessCaptureRequest,
    .get_metadata_vendor_tag_ops        = NULL,
    .dump                               = Camera::sDump,
    .flush                              = Camera::sFlush,
    .reserved = {0}
};

}; /* namespace android */
