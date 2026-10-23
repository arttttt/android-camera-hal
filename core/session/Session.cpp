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

#define LOG_TAG "Cam-Session"

#include "mochacamera/Session.h"

#include <math.h>
#include <stdio.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <functional>
#include <optional>

#include <linux/videodev2.h>
#include <utils/Errors.h>
#include <utils/Log.h>
#include <utils/Timers.h>

#include "CapabilityProbe.h"
#include "CaptureSize.h"
#include "CaptureEngine.h"
#include "DeviceErrorReporter.h"
#include "FinishEngine.h"
#include "Focuser.h"
#include "Frame.h"
#include "FrameRegistry.h"
#include "IspEngine.h"
#include "IspPipeline.h"
#include "JpegEngine.h"
#include "ResultEngine.h"
#include "StatsEngine.h"
#include "V4l2Controls.h"
#include "V4l2Device.h"
#include "SensorSubdev.h"
#include "SensorWriter.h"
#include "V4l2Source.h"
#include "ViController.h"
#include "3a/AutoFocusController.h"
#include "ipa/Ipa3A.h"
#include "jpeg/JpegEncoder.h"
#include "mochacamera/ports/GrallocPort.h"
#include "mochacamera/ports/ResultSink.h"
#include "pipeline/BufferProcessor.h"
#include "pipeline/stages/DemosaicBlitStage.h"
#include "pipeline/stages/ResultDispatchStage.h"
#include "pipeline/stages/StatsProcessStage.h"
#include "sensor/DelayedControls.h"
#include "sensor/ExposureTarget.h"
#include "sensor/SensorConfig.h"
#include "sensor/SensorTuning.h"

namespace mocha::camera {

using android::AutoFocusController;
using android::BufferProcessor;
using android::DelayedControls;
using android::ExposureTarget;
using android::FrameMap;
using android::DemosaicBlitStage;
using android::DeviceErrorReporter;
using android::Focuser;
using android::Ipa;
using android::Ipa3A;
using android::IspPipeline;
using android::JpegEncoder;
using android::Resolution;
using android::ResultDispatchStage;
using android::SensorConfig;
using android::SensorGeometry;
using android::SensorMode;
using android::SensorTuning;
using android::StatsProcessStage;
using android::V4l2Device;
using android::SensorWriter;
using android::ViController;
using android::V4l2Source;
using android::readSensorGeometry;

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

/* camera3.h: flush returns once the HAL holds nothing, and as soon as
 * possible. */
constexpr int kFlushTimeoutMs = 1000;

} /* namespace */

struct Session::Impl {
    Impl(const Config &config, ResultSink *sink, GrallocPort *gralloc);
    ~Impl();

    const Config  config;
    ResultSink   *sink;
    GrallocPort  *gralloc;

    SensorTuning tuning;
    SensorConfig sensorCfg;
    int16_t      ccmQ10[9] = {};

    std::unique_ptr<V4l2Device> dev;
    Focuser                     focuser;

    Capability capability;
    bool       capabilityProbed = false;
    size_t     jpegBufferSize   = 0;

    /* Built on the first open, kept until the session goes. */
    bool infrastructureBuilt = false;
    std::unique_ptr<IspPipeline>         isp;
    std::unique_ptr<AutoFocusController> af;
    std::unique_ptr<JpegEncoder>         jpeg;
    std::unique_ptr<BufferProcessor>     bufferProcessor;
    std::unique_ptr<FrameSource>         frameSource;
    std::unique_ptr<FrameRegistry>       registry;
    std::unique_ptr<DeviceErrorReporter> deviceError;
    std::unique_ptr<SensorWriter>        sensorWriter;
    std::unique_ptr<DemosaicBlitStage>   demosaicBlitStage;
    std::unique_ptr<StatsProcessStage>   statsProcessStage;
    std::unique_ptr<ResultDispatchStage> resultDispatchStage;
    std::unique_ptr<Ipa>                 ipa;
    std::unique_ptr<DelayedControls>     delayedControls;
    /* The IPA's latest exposure decision, for requests in auto AE. */
    ExposureTarget                       exposureTarget;
    /* Null when the VI has no frame wait control. */
    std::unique_ptr<ViController>        viController;
    std::unique_ptr<ResultEngine>        resultEngine;
    std::unique_ptr<JpegEngine>          jpegEngine;
    std::unique_ptr<FinishEngine>        finishEngine;
    std::unique_ptr<IspEngine>           ispEngine;
    std::unique_ptr<StatsEngine>         statsEngine;
    std::unique_ptr<CaptureEngine>       captureEngine;
    bool enginesRunning = false;

    /* The capture size of the running configuration; zero before the
     * first one of a session. */
    Size captureSize;

    /* The pixel array and the modes' windows, read with the capability;
     * and the running mode's frame against the active array, set by
     * configure while the engines are stopped. */
    SensorGeometry geometry;
    FrameMap       frameMap;

    /* AE mode of the previous request, for the manual→auto switch;
     * empty until the first request of the session. */
    std::optional<AeMode> lastAeMode;

    /* Set while flush() waits; a request arriving then is cancelled. */
    std::atomic<bool> flushing{false};

    const Capability &probe();
    void populateSensorConfigFromDriver();
    bool buildInfrastructure();
    void destroyInfrastructure();
    bool startEngines();
    void stopEngines();
    void cancelInFlight(const std::function<bool(const Frame &)> &which);
};

Session::Impl::Impl(const Config &c, ResultSink *s, GrallocPort *g)
    : config(c), sink(s), gralloc(g) {
    tuning.load(config.sensorName.c_str(), config.integrator.c_str());

    /* Sensor-config (frame-length / gain ranges etc.) is a function of
     * the sensor alone; driver-reported limits are merged in on open. */
    sensorCfg = config.facingFront ? SensorConfig::ov5693() : SensorConfig::imx179();
    /* Without a tuning, the same stand-in the static metadata used to
     * fall back on. */
    sensorCfg.focalLengthMm = tuning.isLoaded() ? tuning.module().focalLengthMm : 3.30f;

    dev.reset(new V4l2Device(config.devNode.c_str()));
}

Session::Impl::~Impl() {
    stopEngines();
    destroyInfrastructure();
    dev->disconnect();
}

namespace {

/* The geometry the sensor's driver reports. A driver that reports none
 * leaves each output showing the whole of an array the size of the
 * largest, as before the drivers told. */
SensorGeometry readGeometry(V4l2Device &dev) {
    std::vector<Size> outputs;
    for (const Resolution &r : dev.availableResolutions()) {
        const Size out{r.width, r.height};
        const bool listed = std::any_of(outputs.begin(), outputs.end(),
            [&](const Size &o) { return o.width == out.width && o.height == out.height; });
        if (!listed) outputs.push_back(out);
    }
    if (std::optional<SensorGeometry> g = readSensorGeometry(dev.sensorNode(), outputs))
        return *g;

    const Resolution largest = dev.sensorResolution();
    ALOGW("the sensor's driver reports no geometry: every output taken as "
          "the whole %ux%u", largest.width, largest.height);
    std::vector<SensorMode> modes;
    for (const Size &out : outputs)
        modes.push_back(SensorMode{out, Rect{0, 0, (int32_t)largest.width,
                                             (int32_t)largest.height}});
    return SensorGeometry::fromModes(Size{largest.width, largest.height},
                                     std::move(modes));
}

} /* namespace */

const Capability &Session::Impl::probe() {
    if (!capabilityProbed) {
        geometry         = readGeometry(*dev);
        capability       = probeCapability(*dev, config.facingFront, &tuning,
                                           sensorCfg, geometry,
                                           config.requestsInFlight);
        jpegBufferSize   = capability.jpeg.maxSize;
        capabilityProbed = true;
    }
    return capability;
}

void Session::Impl::populateSensorConfigFromDriver() {
    /* The mode's timing is the driver's: its frame length range starts
     * at the mode's own frame (AE runs there and never stretches it)
     * and tops out at the longest the sensor takes, for manual long
     * exposures. */
    int32_t flMin, flMax, flDef;
    if (dev->queryControl(V4L2_CID_FRAME_LENGTH, &flMin, &flMax, &flDef)) {
        sensorCfg.frameLenDefault = flDef;
        sensorCfg.frameLenMax     = flMax;
        ALOGD("Frame length: def=%d max=%d", flDef, flMax);
    } else {
        ALOGE("V4L2_CID_FRAME_LENGTH query failed: no frame length range");
    }
    int32_t llMin, llMax, llDef;
    if (dev->queryControl(V4L2_CID_LINE_LENGTH, &llMin, &llMax, &llDef)) {
        sensorCfg.lineLengthDefault = llDef;
        sensorCfg.lineLengthMax     = llMax;
        ALOGD("Line length: def=%d max=%d", llDef, llMax);
    } else {
        sensorCfg.lineLengthDefault = 0;
        sensorCfg.lineLengthMax     = 0;
        ALOGW("no V4L2_CID_LINE_LENGTH: exposures stop at the longest frame");
    }
    int32_t lineNs;
    if (dev->getControl(V4L2_CID_LINE_TIME_NS, &lineNs) && lineNs > 0) {
        sensorCfg.lineTimeNs = lineNs;
        ALOGD("Line time: %d ns", lineNs);
    } else {
        ALOGE("V4L2_CID_LINE_TIME_NS read failed: exposure cannot be timed");
    }
    int32_t gMin, gMax, gDef;
    if (dev->queryControl(V4L2_CID_GAIN, &gMin, &gMax, &gDef)) {
        sensorCfg.gainMin     = gMin;
        sensorCfg.gainMax     = gMax;
        sensorCfg.gainDefault = gDef;
        ALOGD("Gain: min=%d max=%d def=%d", gMin, gMax, gDef);
    }
    int32_t eMin, eMax, eDef;
    if (dev->queryControl(V4L2_CID_EXPOSURE, &eMin, &eMax, &eDef)) {
        sensorCfg.exposureMin     = eMin;
        sensorCfg.exposureMax     = eMax;
        sensorCfg.exposureDefault = eDef;
        ALOGD("Exposure: min=%d max=%d def=%d (us)", eMin, eMax, eDef);
    }
}

bool Session::Impl::buildInfrastructure() {
    if (infrastructureBuilt) return true;

    isp.reset(android::createIspPipeline());
    if (!isp || !isp->init()) {
        ALOGE("ISP init failed");
        isp.reset();
        return false;
    }
    isp->setEnabled(config.softIsp);

    /* Optical-black bias — set on the ISP here and on the stats engine
     * below. Shared between the demosaic shader and the CPU stats
     * encoder so both read the same signal space; mismatched bias on
     * the stats side lets the offset dominate raw means on dark scenes
     * and collapses the R/G/B ratios gray-world AWB needs. */
    if (tuning.isLoaded())
        isp->setBlackLevel((uint32_t)tuning.opticalBlack().r);

    /* WB prior + seed CCM. Takes the hottest-CCT CcmSet the tuning
     * ships as the sensor's calibrated "daylight" anchor. Ipa3A stores
     * the same prior as its last WB so the very first frame (before any
     * stats arrive) renders through sensor-correct WB gains; the ccmQ10
     * buffer handed to setCcm is then rewritten every AWB tick via
     * Ipa3A's U → CCT → LERP path. Tunings without awb.v4 fall back to
     * nearest-CCT pick at the same daylight anchor.
     *
     * `wbGainPrior` carries ready-to-apply shader gains — (G/R, 1.0,
     * G/B) of the raw anchor, post black-level. The CCM seed needs the
     * same U = ln(G/B) the per-frame AWB loop uses: wbGainPrior[2]. */
    float wbGainPrior[3];
    tuning.defaultWbGain(wbGainPrior);
    if (tuning.awbParams().loaded && wbGainPrior[2] > 0.f) {
        const float uSeed = logf(wbGainPrior[2]);
        tuning.ccmForCctLerpQ10(tuning.estimateCctFromU(uSeed), ccmQ10);
    } else if (!tuning.ccmSets().empty()) {
        int daylightCctK = tuning.ccmSets().front().cctK;
        for (const auto &set : tuning.ccmSets())
            if (set.cctK > daylightCctK) daylightCctK = set.cctK;
        tuning.ccmForCctQ10(daylightCctK, ccmQ10);
    }
    isp->setCcm(ccmQ10);

    /* AF before the IPA, which drives it from its tick -- only for a
     * module whose lens open() found: without one AF has nothing to
     * move, and the frame ROI stays the centre default. */
    if (config.softIsp && focuser.isOpen()) {
        af.reset(new AutoFocusController(&tuning, &frameMap));
    }

    {
        /* The one place the lens is moved from: AF decides, the IPA
         * coordinator writes, on the IPA's own thread. */
        std::function<void(int32_t)> lensWriter;
        if (af) {
            Focuser *lens = &focuser;
            lensWriter = [lens](int32_t pos) { lens->setPosition(pos); };
        }
        ipa.reset(new Ipa3A(sensorCfg, isp.get(), af.get(), std::move(lensWriter),
                            &tuning, wbGainPrior, ccmQ10, probe().ae.evStep));
    }

    jpeg.reset(new JpegEncoder());

    {
        BufferProcessor::Deps d;
        d.recorder       = isp.get();
        d.jpegRing       = isp.get();
        d.yuv            = isp.get();
        d.jpeg           = jpeg.get();
        d.gralloc        = gralloc;
        d.jpegBufferSize = &jpegBufferSize;
        d.sensorCfg      = &sensorCfg;
        d.make           = config.make;
        d.model          = config.model;
        d.fNumber        = probe().lens.aperture;
        d.focalLengthMm  = sensorCfg.focalLengthMm;
        bufferProcessor.reset(new BufferProcessor(d));
    }

    frameSource.reset(new V4l2Source(dev.get()));
    registry.reset(new FrameRegistry(kFrameCeiling));
    deviceError.reset(new DeviceErrorReporter(sink));
    statsEngine.reset(new StatsEngine());
    if (tuning.isLoaded())
        statsEngine->setBlackLevel((uint32_t)tuning.opticalBlack().r);

    /* DelayedControls: what requests want of the sensor, by the sensor
     * frame it is in effect on; the capture engine files, writes and
     * reads it. */
    {
        DelayedControls::Config cfg;
        for (int i = 0; i < DelayedControls::COUNT; ++i)
            cfg.delay[i] = sensorCfg.controlDelay[i];
        delayedControls.reset(new DelayedControls(cfg));
    }

    /* The VI's frame wait, kept to the frame lengths written. */
    {
        int32_t mn, mx, df;
        if (dev->queryControl(V4L2_CID_VI_CAPTURE_TIMEOUT, &mn, &mx, &df)) {
            V4l2Device *d = dev.get();
            viController.reset(new ViController(
                [d](int32_t us) {
                    if (!d->setControl(V4L2_CID_VI_CAPTURE_TIMEOUT, us))
                        ALOGE("VI capture timeout %d us not set", us);
                },
                sensorCfg.controlDelay[DelayedControls::FRAME_LENGTH], mn, mx));
        } else {
            ALOGW("VI has no capture timeout control; it waits its default");
        }
    }

    /* What each frame's tick writes reaches the sensor through it. */
    sensorWriter.reset(new SensorWriter(dev.get(), &sensorCfg,
                                        viController.get()));

    {
        DemosaicBlitStage::Deps d;
        d.bufferProcessor = bufferProcessor.get();
        d.source          = frameSource.get();
        d.recorder        = isp.get();
        d.jpegRing        = isp.get();
        d.deviceError     = deviceError.get();
        demosaicBlitStage.reset(new DemosaicBlitStage(d));
    }
    {
        ResultDispatchStage::Deps d;
        d.sink            = sink;
        d.af              = af.get();
        d.sensorCfg       = &sensorCfg;
        resultDispatchStage.reset(new ResultDispatchStage(d));
    }
    {
        StatsProcessStage::Deps d;
        d.ipa             = ipa.get();
        d.exposureTarget  = &exposureTarget;
        d.sensorCfg       = &sensorCfg;
        d.stats           = statsEngine.get();
        d.af              = af.get();
        d.sink            = sink;
        statsProcessStage.reset(new StatsProcessStage(d));
    }

    /* The engines, downstream first: each takes the one after it. */
    {
        ResultEngine::Deps d;
        d.dispatch = resultDispatchStage.get();
        d.registry = registry.get();
        resultEngine.reset(new ResultEngine(d));
    }
    jpegEngine.reset(new JpegEngine());
    {
        FinishEngine::Deps d;
        d.bufferProcessor = bufferProcessor.get();
        d.statsProcess    = statsProcessStage.get();
        d.jpeg            = jpegEngine.get();
        finishEngine.reset(new FinishEngine(d));
    }
    {
        IspEngine::Deps d;
        d.demosaic    = demosaicBlitStage.get();
        d.finish      = finishEngine.get();
        d.deviceError = deviceError.get();
        ispEngine.reset(new IspEngine(d));
    }
    {
        CaptureEngine::Deps d;
        d.source      = frameSource.get();
        d.inputRing   = isp.get();
        d.af          = af.get();
        d.frameMap    = &frameMap;
        d.sink        = sink;
        d.controls    = delayedControls.get();
        d.writer      = sensorWriter.get();
        d.sensorCfg   = &sensorCfg;
        d.exposureTarget = &exposureTarget;
        d.evStep      = probe().ae.evStep;
        d.writeDelayNs = (int64_t)config.writeDelayUs * 1000;
        d.demosaic    = ispEngine.get();
        d.stats       = statsEngine.get();
        d.deviceError = deviceError.get();
        captureEngine.reset(new CaptureEngine(d));
    }

    infrastructureBuilt = true;
    return true;
}

void Session::Impl::destroyInfrastructure() {
    captureEngine.reset();
    ispEngine.reset();
    finishEngine.reset();
    jpegEngine.reset();
    resultEngine.reset();
    statsProcessStage.reset();
    resultDispatchStage.reset();
    demosaicBlitStage.reset();
    sensorWriter.reset();
    statsEngine.reset();
    delayedControls.reset();
    viController.reset();
    ipa.reset();
    registry.reset();
    deviceError.reset();
    frameSource.reset();
    bufferProcessor.reset();
    af.reset();
    jpeg.reset();
    if (isp) {
        isp->destroy();
        isp.reset();
    }
    infrastructureBuilt = false;
}

void Session::Impl::cancelInFlight(const std::function<bool(const Frame &)> &which) {
    if (registry)      registry->cancelIf(which);
    if (captureEngine) captureEngine->wake();
}

void Session::Impl::stopEngines() {
    if (!enginesRunning) return;
    enginesRunning = false;

    /* From the source towards the result. A stopped engine lets the
     * jobs it holds go, and with them their frames' tokens: those
     * frames go on to the result engine and out as failed requests.
     * The demosaic engine first gives a frame on the GPU its fence's
     * time. */
    captureEngine->stop();
    statsEngine->stop();
    ispEngine->stop();
    finishEngine->stop();

    /* Frames past the GPU -- committed, waiting on a JPEG or on their
     * turn -- go out normally if they can. */
    if (!registry->waitEmpty(kStopDrainTimeoutMs))
        ALOGE("stopEngines: %zu frames still held after %d ms",
              registry->count(), kStopDrainTimeoutMs);
    jpegEngine->stop();
    resultEngine->stop();

    /* Whatever is left goes out in error, from here; no engine runs. */
    for (const FrameRef &frame : registry->takeAll())
        resultEngine->dispatchFailed(*frame);

    frameSource->stop();
}

bool Session::Impl::startEngines() {
    if (enginesRunning) return true;
    if (!infrastructureBuilt) return false;

    /* Downstream first, so no engine hands work to one not yet
     * running; the source before the capture engine that drives it. */
    const bool started = frameSource->start()
                      && resultEngine->start()
                      && jpegEngine->start()
                      && finishEngine->start()
                      && ispEngine->start()
                      && statsEngine->start()
                      && captureEngine->start();
    if (!started) {
        ALOGE("startEngines: an engine did not start; stopping the rest");
        enginesRunning = true;
        stopEngines();
        return false;
    }
    enginesRunning = true;
    return true;
}

/* ------------------------------------------------------------------ */

Session::Session(const Config &config, ResultSink *sink, GrallocPort *gralloc)
    : impl(std::make_unique<Impl>(config, sink, gralloc)) {}

Session::~Session() = default;

const Capability &Session::capability() {
    return impl->probe();
}

int32_t Session::sensorOrientationDegrees() const {
    /* From the per-module tuning; 0 when it did not load -- wrong but
     * not crashing, and a missing tuning already degrades the HAL. */
    return impl->tuning.module().sensorOrientationDegrees;
}

bool Session::open() {
    Impl &s = *impl;
    s.dev->connect();

    /* The lens, if the device has one: the media graph links it into
     * the sensor -- no paths or hardware ids in config. */
    ALOGI("open: front=%d lens=%d", (int)s.config.facingFront, (int)s.dev->hasLens());
    if (s.dev->hasLens() && !s.focuser.open(s.dev->lensNode()))
        ALOGW("open: the lens did not open; AF will be inactive");

    /* Driver-advertised ranges into the sensor config before anything
     * consumes it: Ipa3A's constructor seeds AE from them. */
    s.populateSensorConfigFromDriver();

    /* The first open builds what the session keeps across close /
     * reopen. */
    return s.buildInfrastructure();
}

void Session::close() {
    Impl &s = *impl;
    s.stopEngines();

    /* Per-session state goes, so the next open starts clean; the
     * infrastructure stays. */
    s.captureSize = Size{};
    s.lastAeMode.reset();
    if (s.af)                 s.af->reset();
    if (s.deviceError)        s.deviceError->rearm();
    if (s.isp)                s.isp->onSessionClose();
    if (s.ipa)                s.ipa->reset();
    if (s.delayedControls)
        s.delayedControls->reset(s.sensorCfg.configureDefaults());
    s.exposureTarget.reset();
    if (s.statsEngine)        s.statsEngine->reset();

    s.dev->disconnect();
}

Session::ConfigureResult Session::configure(std::span<const StreamSpec> outputs) {
    Impl &s = *impl;
    if (!s.infrastructureBuilt) return ConfigureResult::Failed;

    std::optional<Size> running;
    if (s.enginesRunning) running = s.captureSize;
    const std::optional<Size> chosen = chooseCaptureSize(s.probe(), outputs, running);
    if (!chosen) return ConfigureResult::Unsupported;
    const Size capture = *chosen;

    /* Skip the V4L2/Vulkan reinit when the capture size has not
     * changed. Apps like Open Camera reissue configure_streams on
     * AE_MODE auto↔manual toggles even when nothing about the streams
     * really changed; the full stop + STREAMOFF + prewarm + STREAMON +
     * start cycle costs ~3 s on Tegra K1 and shows as the camera
     * "reopening". New output streams build their gralloc cache
     * entries lazily on first blit. */
    if (s.enginesRunning && s.captureSize.width == capture.width &&
        s.captureSize.height == capture.height) {
        ALOGD("configure: same %ux%u capture as last — no reinit",
              capture.width, capture.height);
        return ConfigureResult::Ok;
    }

    /* Every frame held goes back first; requests come again once the
     * streams are set. */
    s.stopEngines();
    s.captureSize = Size{};

    /* Drop the stats cycle in progress before the ISP ring gets resized
     * out from under it. */
    s.statsEngine->reset();

    ALOGD("capture %ux%u, soft_isp=%d", capture.width, capture.height,
          (int)s.config.softIsp);

    if (!s.dev->setStreaming(false)) {
        ALOGE("Could not stop streaming");
        return ConfigureResult::Failed;
    }

    /* Allocate the ISP's input buffers at the capture size before V4L2
     * so their dma-buf fds can go to VIDIOC_REQBUFS(DMABUF). If any
     * export fails, MMAP + memcpy instead. */
    s.isp->prewarm(capture.width, capture.height, s.dev->pixelFormat());
    {
        const int n = s.isp->inputBufferCount();
        if (n > 0 && n == V4L2DEVICE_BUF_COUNT) {
            int fds[V4L2DEVICE_BUF_COUNT];
            bool allOk = true;
            for (int i = 0; i < n; i++) {
                fds[i] = s.isp->exportInputBufferFd(i);
                if (fds[i] < 0) { allOk = false; break; }
            }
            if (allOk) {
                s.dev->setDmaBufFds(fds, n);
                ALOGD("V4L2 input: DMABUF mode, %d slots", n);
            } else {
                for (int i = 0; i < n; i++) if (fds[i] >= 0) ::close(fds[i]);
                ALOGW("V4L2 input: DMABUF export failed, falling back to MMAP");
            }
        }
    }

    if (!s.dev->setResolution(capture.width, capture.height)) {
        ALOGE("Could not set resolution");
        return ConfigureResult::Failed;
    }

    /* The mode's timing, as the driver set it for this format. */
    s.populateSensorConfigFromDriver();

    /* What of the active array the mode's frame shows. */
    if (const std::optional<SensorMode> mode = s.geometry.modeFor(capture)) {
        s.frameMap = FrameMap(s.geometry.activeArray, *mode);
        const Rect v = s.frameMap.visible();
        ALOGD("capture %ux%u shows (%d,%d) %dx%d of the active array",
              capture.width, capture.height, v.left, v.top, v.width, v.height);
    } else {
        ALOGW("capture %ux%u: the sensor reports no window for it",
              capture.width, capture.height);
        s.frameMap = FrameMap();
    }

    /* The mode's own frame and the exposure / gain defaults, so the
     * sensor has a sane baseline before STREAMON; the IPA takes over
     * from frame 1. The frame and line length too: the driver keeps
     * the last written, which a manual long exposure may have
     * stretched. */
    if (s.viController)
        s.viController->startStream(s.sensorCfg.frameDurationNs(
            s.sensorCfg.frameLenDefault, s.sensorCfg.lineLengthDefault));
    s.dev->setControl(V4L2_CID_FRAME_LENGTH, s.sensorCfg.frameLenDefault);
    if (s.sensorCfg.lineLengthMax > 0)
        s.dev->setControl(V4L2_CID_LINE_LENGTH, s.sensorCfg.lineLengthDefault);
    s.dev->setControl(V4L2_CID_EXPOSURE,     s.sensorCfg.exposureDefault);
    s.dev->setControl(V4L2_CID_GAIN,         s.sensorCfg.gainDefault);
    /* The stream's frames run on these until the first write lands. */
    s.delayedControls->reset(s.sensorCfg.configureDefaults());

    if (!s.dev->setStreaming(true)) {
        ALOGE("Could not start streaming");
        return ConfigureResult::Failed;
    }

    if (!s.startEngines()) {
        ALOGE("configure: the engines did not start");
        return ConfigureResult::Failed;
    }
    s.captureSize = capture;
    return ConfigureResult::Ok;
}

bool Session::running() const {
    return impl->enginesRunning;
}

void Session::submit(FrameRequest request) {
    Impl &s = *impl;

    auto frame = std::make_shared<Frame>();
    frame->frameNumber = request.frameNumber;
    frame->settings    = request.settings;
    frame->tAccepted   = systemTime();
    frame->outputs.resize(request.outputs.size());
    for (size_t i = 0; i < request.outputs.size(); ++i) {
        frame->outputs[i].stream = request.outputs[i].stream;
        frame->outputs[i].buffer = request.outputs[i].buffer;
        frame->outputs[i].acquireFence.reset(request.outputs[i].acquireFence);
    }

    /* Open Camera and similar Camera2 apps don't issue an explicit
     * flush() on AE manual→auto toggles, so whatever long manual
     * exposure was in flight would drain naturally — observable as
     * "manual→auto takes seconds" in preview. The transition gives up
     * the frames still waiting for their Bayer, through the same
     * cancellation flush() uses. A frame that already has its Bayer is
     * past the wait and completes; a snapshot is never given up.
     * Auto→manual has nothing long to drain. */
    const AeMode aeMode = frame->settings.ae.mode;
    if (s.lastAeMode == AeMode::Off && aeMode != AeMode::Off) {
        ALOGD("AE off→on, cancelling frames waiting for Bayer");
        s.cancelInFlight([](const Frame &f) {
            return !f.settings.stillCapture &&
                   !f.hasBayer.load(std::memory_order_acquire);
        });
    }
    s.lastAeMode = aeMode;

    /* The frame's first token, taken before the result engine sees the
     * frame: it waits for the frame until the last token goes. */
    frame->settled = s.resultEngine->settledWake();
    FrameToken token(frame);

    if (!s.registry->add(frame)) {
        /* More frames held than a framework keeping its stream contract
         * can send. Camera3 gives no return code for that; the request
         * goes out at once as a failed one, in order. */
        ALOGE("frame %u: over the ceiling of %zu frames held — failed",
              frame->frameNumber, kFrameCeiling);
        frame->status.fail(android::NO_MEMORY);
    }
    /* A request arriving while flush() waits is cancelled like the ones
     * in flight. flush sets the flag before cancelling the registry, so
     * a frame added after that cancel sees it. */
    if (s.flushing.load()) frame->status.cancel();

    /* Results go out in the order frames reach the result engine: the
     * order requests arrive in. The engines run (running() is the
     * caller's precondition), so it takes the frame. */
    if (!s.resultEngine->submit(frame)) {
        ALOGE("frame %u: the result engine is stopped", frame->frameNumber);
        s.registry->remove(frame->frameNumber);
        return;
    }
    /* A capture engine that refuses lets the token go; the frame goes
     * out as a failed request. */
    s.captureEngine->submit(std::move(token));
}

void Session::flush() {
    Impl &s = *impl;
    if (!s.registry) return;

    /* camera3.h: return only once the HAL holds no more requests or
     * buffers, and as soon as possible. Every frame held is cancelled
     * (one already failed or committed keeps its outcome); each engine
     * looks at the status before its work and lets a cancelled frame
     * go, and the result engine sends it out as a failed request.
     * Committed frames, too late to cancel, complete normally. The
     * capture engine is woken, so a frame waiting on a long manual
     * exposure is let go now rather than a frame period later. The
     * capture in progress finishes naturally; no STREAMOFF. */
    ALOGD("flush: inFlight=%zu", s.registry->count());
    s.flushing.store(true);
    s.cancelInFlight([](const Frame &) { return true; });

    const bool drained = s.registry->waitEmpty(kFlushTimeoutMs);
    s.flushing.store(false);
    if (!drained) {
        ALOGE("flush: %zu requests still in flight after %d ms",
              s.registry->count(), kFlushTimeoutMs);
    }
}

void Session::dump(int fd) {
    Impl &s = *impl;
    dprintf(fd, "  core: %s, capture %ux%u, %zu frames held, soft ISP %s\n",
            s.enginesRunning ? "running" : "stopped",
            s.captureSize.width, s.captureSize.height,
            s.registry ? s.registry->count() : (size_t)0,
            s.config.softIsp ? "on" : "off");
}

} /* namespace mocha::camera */
