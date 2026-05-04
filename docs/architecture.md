# Architecture

Targets the Xiaomi Mi Pad 1 (Tegra K1, codename `mocha`) running
LineageOS 14.1 / Android 7.1.2. Bayer-only Vulkan ISP is the one and
only data path; the HAL has no CPU fallback.

The pipeline is **asynchronous and event-driven**: framework calls
return in < 1 ms, all blocking work (V4L2 dequeue, GPU submit fence,
libjpeg encode, NEON statistics) lives on its own worker thread, and
threads communicate through bounded queues + sync_fd / eventfd.

For the design rationale and PR-level history of the async refactor,
see [tier3_architecture.md](tier3_architecture.md). This document
describes what the codebase does **today**.

## Component overview

```
                      Android camera framework (cameraserver)
                                     │ Camera3 ops
                            ┌────────▼────────┐
                            │     Camera      │  hal/Camera.cpp
                            │  (Camera3 dev)  │
                            └─┬──────┬──────┬─┘
                              │      │      │
                ┌─────────────┘      │      └──────────────┐
                ▼                    ▼                     ▼
          RequestThread       per-camera helpers    V4l2Source / CaptureThread
       (hal/pipeline/)        (hal/3a/, hal/jpeg/)  (hal/pipeline/v4l2/)
                                                          │ /dev/video0 + DMABUF
                                                          ▼
                                                  ┌──────────────────┐
                                                  │ VulkanIspPipeline│
                                                  │ (isp/vulkan/)    │
                                                  └─┬────────────────┘
                                                    │ produce-once submit
                                                    │
                          ┌─────────────────────────┴───────────────┐
                          │                  │                      │
                          ▼                  ▼                      ▼
                       gralloc      JpegSnapshot ring         NV12 mapped buffer
                     (preview /          │                          │
                      video frames)      ▼                          ▼
                                    JpegWorker (libjpeg)    BufferProcessor
                                         │                  (NV12→YV12/I420 repack)
                                         ▼
                              ResultThread → process_capture_result
```

`V4l2Device` hands `VulkanIspPipeline`-exported dma-buf fds to
`VIDIOC_QBUF` so the VI DMA writes captured Bayer directly into the
GPU-visible input ring — no CPU copy on the hot path.

## Top-level objects

- **`Camera`** (`hal/Camera.cpp/.h`) — one instance per physical
  camera. Implements the Camera3 device ops
  (`process_capture_request`, `configure_streams`,
  `construct_default_request_settings`, `flush`,
  `register_stream_buffers = NULL`) and owns the worker threads and
  per-camera helpers below. Build-once lifecycle: helpers and threads
  are created on the first `openDevice` and survive
  `closeDevice → openDevice` of subsequent sessions; only V4L2
  reconfig and tracker drain happen per-session.

- **Per-camera helpers** under `hal/`:
  - `hal/3a/AutoFocusController` — CDAF state machine, AF region
    parsing. Pure return-style: `process(stats, aeConverged) →
    AfResult`; the coordinator translates `startSweep` /
    `sweepComplete` flags into AE / AWB lock toggles.
  - `hal/3a/AutoExposureController` — EV-space LPF + dead-band +
    asymmetric speed + highlight-protection candidate. Pure return-
    style: `process(stats, meta, currentWb) → AeResult`. Static
    helper `parseManualSettings` handles the `AE_MODE = OFF` /
    cold-start V4L2 triple computation.
  - `hal/3a/Awb` — pure-virtual AWB interface. Two impls:
    `GrayWorldAwbController` (fallback / sensors without bayes
    calibration) keeps the original 96-patch confidence gate +
    EMA-relax to prior + CCT-LERP CCM behaviour;
    `BayesianAwbController` is the production path on IMX179 —
    coarseSearch over CT curves + lux-conditioned prior +
    deltaLimit clamp + biasCT pull + off-curve fineSearch + IIR
    damping. Selected by `hal/3a/AwbFactory::createAwb` from the
    tuning's `active.awb.algorithm` flag plus a populated
    `bayes` block. Same return-style:
    `process(stats, luxIndex, awbMode) → AwbResult`,
    `applyManualGains(...)` for `AWB_MODE = OFF`.
  - `hal/ipa/Ipa3A` — 3A coordinator. Owns the three controllers,
    runs gating decisions, dispatches in order
    AWB → AE → AF, routes their results to backends
    (`IspPipeline::setWbGains` / `mCcmQ10` for AWB, `DelayedControls`
    for AE, `V4l2Device` focuser for AF). Emits AWB and AE partial
    results inside the IPA tick (counters 1, 2); the final partial
    with buffers comes from `ResultDispatchStage` (counter 3).
  - `hal/ipa/StatsWorker` + `hal/ipa/NeonStatsEncoder` — NEON
    statistics worker thread + the kernel that computes
    `IpaStats::rgbMean` and `IpaStats::focusMetric` over raw Bayer.
  - `hal/jpeg/JpegEncoder` (a `PostProcessor`) — libjpeg + EXIF
    Orientation marker.
  - `hal/pipeline/StreamConfig` — stream-list normalisation +
    V4L2 capture resolution pick.
  - `hal/pipeline/BufferProcessor` — per-output-buffer dispatch
    (RGBA blit / YUV blit / BLOB encode).
  - `hal/metadata/{CameraStaticMetadata,RequestTemplateBuilder,
    ResultMetadataBuilder}` — Camera3 metadata builders.

- **`V4l2Device`** (`v4l2/V4l2Device.cpp/.h`) — thin C++ wrapper
  over `/dev/video0`. Speaks `V4L2_MEMORY_DMABUF` (the production
  path); `setDmaBufFds()` switches into DMABUF mode with caller-
  supplied capture fds. Manages `VIDIOC_QBUF` / `VIDIOC_DQBUF`,
  `VIDIOC_S_EXT_CTRLS` (controls grouped by class so the kernel's
  per-class drain doesn't lose updates), `VIDIOC_QUERYCTRL` for
  range introspection. Also opens the focuser subdev
  (`/dev/v4l-subdev*`) when found.

- **`VulkanIspPipeline`** (`isp/vulkan/VulkanIspPipeline.cpp/.h`) —
  the only `IspPipeline` impl. Compute demosaic into a per-slot
  scratch ring + fragment ROP blit + libjpeg-feeding host-mapped
  ring + RGBA→NV12 compute encoder. See
  [isp-pipeline.md](isp-pipeline.md) for the per-frame mechanics.

- **`Ipa`** abstraction in `hal/ipa/Ipa.h` with `Ipa3A` and a
  `StubIpa` for cold-bringup.

- **`DelayedControls`** (`isp/sensor/DelayedControls.cpp/.h`) —
  ring of pending exposure / gain writes tagged with
  `frameNumber + controlDelay[id]` so per-frame result metadata
  reports the value that actually applied at silicon, not the
  one the request asked for.

## Thread topology

| Thread          | Owner                       | Blocks on                       | Responsibility                                                                |
|-----------------|-----------------------------|---------------------------------|--------------------------------------------------------------------------------|
| Main (binder)   | framework                   | nothing                         | Camera3 ops entry; `processCaptureRequest` deep-copies into `PipelineContext`, pushes, returns 0 |
| RequestThread   | `Camera`                    | condvar on queue                | Pop ctx, run ApplySettings + ShutterNotify + Capture, push to PipelineThread   |
| CaptureThread   | `V4l2Source`                | `poll()` on V4L2 fd + eventfd   | DQBUF / QBUF, drain-to-latest, signal `bayerReady(slot)`                       |
| StatsWorker     | `Camera`                    | `poll()` on job eventfd + stopfd| NEON reduce of raw Bayer into `IpaStats`; progressive over `phaseCount` submits|
| PipelineThread  | `Camera`                    | `poll()` on fence fds + eventfd | Vulkan record + submit, fence fan-out, `StatsProcessStage` (IPA call)          |
| ResultThread    | `Camera`                    | condvar on completion queue     | `process_capture_result` dispatch, BLOB FIFO gate, in-flight tracker remove   |
| JpegWorker      | `Camera`                    | condvar on JPEG queue           | libjpeg encode async, releases the JpegSnapshot ring slot when done           |

The framework callback ordering rule (`process_capture_result` must
fire in monotonic `frame_number`) is enforced by single-thread
serialisation in `ResultThread` plus a `PipelineContext::jpegPending`
gate that holds back preview-only ctxs that follow a BLOB ctx still
in `JpegWorker`.

`stopWorkers()` order on session close: BayerSource → RequestThread
→ StatsWorker → PipelineThread → JpegWorker → ResultThread. The
order is load-bearing — stopping PipelineThread before JpegWorker
guarantees no new JPEG jobs arrive after the worker has quiesced.
`startWorkers()` is the reverse.

## Request lifecycle

```
processCaptureRequest(req)                              [binder]
    │ deep-copy req → PipelineContext, push
    ▼
RequestQueue                                             [bounded, depth ≈ 4]
    │
    ▼
RequestThread                                            [request-side stages]
    ApplySettingsStage   parse settings, push to DelayedControls,
                          V4L2 setControls(batch)
    ShutterNotifyStage   tShutter = systemTime(); notify SHUTTER
    CaptureStage         BayerSource::acquireNextFrame()
                          → ctx.bayerFrame = slot from V4l2Source
    DemosaicBlitStage    isp.beginFrame(slot); for each output buffer:
                            isp.blitToGralloc / blitToYuv / blitToJpegCpu
                          isp.endFrame() → submit fence fd
                          push to PipelineQueue
    │
    ▼
PipelineQueue
    │
    ▼
PipelineThread                                           [post-submit stages]
    ↳ poll() on submit fence fd
    StatsDispatchStage   StatsWorker::submit(bayer slot, focusRoi)
                          (StatsWorker computes IpaStats off-thread)
    StatsProcessStage    StatsWorker::peek(stats) → Ipa3A.processStats
                          → AWB / AE partials emitted from the IPA tick
                          → Batch{exposure,gain} → DelayedControls.push
                          → AutoFocusController::process(stats, aeConverged)
                            inside Ipa3A; routes startSweep / sweepComplete
                            into AE / AWB lock toggles
    push to ResultQueue
    │
    ▼
ResultQueue
    │
    ▼
ResultThread
    ↳ wait for ctx.jpegPending == 0
    ResultDispatchStage  build result metadata (RequestMetadataEcho +
                          per-frame 3A state), call
                          camera3_callback_ops::process_capture_result
                          flush Bayer slot back to V4l2Source
                          InFlightTracker::removeBySequence
```

JpegWorker runs in parallel: when `DemosaicBlitStage` records a BLOB
output via `blitToJpegCpu`, it acquires a host-mapped snapshot ring
slot. After the GPU submit fence signals,
`PostProcessStage` (on PipelineThread) hands the snapshot to
`JpegWorker`. The worker libjpeg-encodes into the BLOB gralloc and
clears `ctx.jpegPending` so ResultThread can release the ctx.

There is **no CPU RGBA fallback**: a `blitToGralloc` failure is a
hardware / driver error and propagates as a per-buffer error +
`notify(ERROR_BUFFER)`. Packed-YUV sensors (UYVY / YUYV) are not
supported — the target hardware is Bayer only.

## Stream configuration

`Camera::configureStreams()` runs inside the binder thread, holding
the global mutex. It:

1. **`StreamConfig::normalize()`** validates and rewrites the stream
   list:
   - HAL3.3: `operation_mode == NORMAL_MODE`,
     `stream_t::rotation == ROTATION_0`,
     `data_space != DEPTH` — non-conformant configs are rejected
     with `BAD_VALUE`.
   - One INPUT/BIDIRECTIONAL stream max (today no INPUT path is
     wired — reserved for ZSL / reprocess).
   - `IMPLEMENTATION_DEFINED` resolves by usage:
     `HW_VIDEO_ENCODER` → `YCbCr_420_888`, otherwise → `RGBA_8888`.
   - Output size cap: 16:9 ≤ 1920×1080. Auxiliary system streams
     (face-detect input, MediaRecorder thumbnail callback) accepted
     up to 1920×1080 at any aspect.
   - Consumer gralloc usage flags preserved; the HAL only adds
     `SW_WRITE_OFTEN` for BLOB outputs that libjpeg fills CPU-side.
   - V4L2 capture resolution: an `HW_VIDEO_ENCODER` stream wins
     (so the sensor locks to the matching FPS mode); else the
     largest non-BLOB stream; else the largest BLOB.

2. **`stopWorkers()`** drains the in-flight tracker via
   `errorCompletePendingRequests` so ctxs upstream of PipelineThread
   surface as errors instead of leaving the framework's output
   buffers parked on the HAL side.

3. **V4L2 reconfig** — `V4l2Device::setStreaming(false)`,
   `setResolution()` (`VIDIOC_S_FMT` + `REQBUFS(DMABUF)`),
   `setStreaming(true)` re-queues every slot before `STREAMON` (the
   kernel returns slots to USERSPACE on `STREAMOFF`).

4. **`startWorkers()`** brings the threads back live in the
   reverse-of-stop order.

No per-stream output-buffer allocation happens in the HAL — gralloc
buffers come from the framework. The HAL owns the V4L2 capture
buffers (DMABUF mode, fds exported from the Vulkan input ring).

## Static characteristics

Built once per camera by `CameraStaticMetadata::build()` in
`hal/metadata/`. Keys grouped by source:

- **From `V4l2Device`:** sensor pixel array size, available
  resolutions (filtered to 16:9 ≤ 1080p), per-mode min frame
  durations, available FPS ranges (from
  `VIDIOC_ENUM_FRAMEINTERVALS`).
- **From `SensorTuning::module`:** physical sensor size, focal
  length, min focus distance, sensor orientation, Bayer pattern.
- **From `SensorConfig` (driver-queried at startup):** exposure /
  gain ranges, base ISO anchor (`kIsoAtUnityGain = 100`).
- **HAL constants:** `device_version=3.4`,
  `hardware_level=LIMITED`, `capabilities=[BACKWARD_COMPATIBLE]`,
  `pipeline_max_depth=4`, `partial_result_count=1`,
  `sync_max_latency=UNKNOWN`,
  per-stage `AVAILABLE_*_MODES`, `AVAILABLE_*_KEYS` arrays.

`Camera` caches the result of the first call.

## 3A summary

- **AE** lives in `Ipa3A::processStats`. State is `filteredTotalUs`
  (absolute exposure × gain at unity, in µs). Each frame two
  candidate ratios are computed: `ratioMean = setpoint / lumaInRoi`
  (spot mean of `IpaStats::rgbMean[*][*][1]` inside `meta.focusRoi`,
  setpoint = `pow(MeanAlg.target/255, 2.2)`) and `ratioHighlight =
  highlightCap / IQM_top2%` (post-WB max-of-channels of the brightest
  ~2 % of patches in the same ROI, cap = 0.8). Strictest wins;
  `target = filteredTotalUs × ratio`, then a single-pole LPF brings
  `filteredTotalUs` toward `target`. Both ratios are clamped to the
  per-sensor `MaxFstopDelta{Pos,Neg}` envelope as a per-frame rate
  limit. A 2 % absolute-deviation dead-band freezes the LPF at
  convergence. Optional asymmetric LPF speed (faster pole when
  `|target/filtered − 1| < closeSpeedZone`) is configurable per
  sensor via `active.hal_overrides.ae.close_speed_zone`; zero / key
  missing disables the boost. EV-comp is a multiplicative offset on
  the target. AE_LOCK holds the converged operating point with an
  EMA-smoothed locked-bias so EV steps under lock don't tear the
  rolling-shutter readout. Exposure / gain split via
  `SensorConfig::splitExposureGain` (prefers exposure up to one
  default frame_length, then gain).
- **AWB** has two implementations behind a common `Awb`
  interface, picked per-sensor by `AwbFactory::createAwb` from
  the tuning JSON's `active.awb.algorithm` flag.

  *GrayWorldAwbController* — original gray-world over
  `rgbMean[16][16][3]` with patch-level saturation / noise-floor
  filtering. A 96-valid-patch confidence gate (`awbMinValidPatches
  = 96`, 37.5 % of the grid) protects against gray-world failures
  on dim / colour-biased scenes: below the gate `lastWb`
  EMA-relaxes back to `wbGainPrior` (the per-CCT calibrated
  daylight neutral). CCT estimated from the gray-world G/B ratio
  via the tuning's `awb.v4` U → CCT polynomial. Used as the
  fallback path on sensors without a Bayes calibration block.
  OV5693 (front) currently runs here.

  *BayesianAwbController* — RPi-style two-stage estimator over
  the calibrated CT curves (`bayes.ctCurveR / ctCurveB`).
  coarseSearch is a log-stepped traversal of the curve domain;
  cost at each candidate t is `Σ_zones min(δ², deltaLimit) +
  biasWeight · min(δ_bias², deltaLimit) − prior(t | luxIndex)`,
  with zones = (R/G, B/G) per filtered patch and
  `(gainR, gainB) = (1/ctR(t), 1/ctB(t))`. The lux-conditioned
  prior is interpolated between calibrated `bayes.priors[]`
  entries; the bias term anchors weak-signal scenes toward
  `biasCT`; the per-zone deltaLimit clamp keeps a single
  off-grey region from dominating. fineSearch refines the
  on-curve winner perpendicular to the curve in (R/G, B/G)
  space (magenta ↔ green correction off the Planckian locus,
  bounded by `transversePos / transverseNeg`). Output gains are
  IIR-smoothed at `damping` rate after the first `startupFrames`
  cold-start snap. Manual AWB presets (`INCANDESCENT` / `DAYLIGHT`
  / etc.) clip the coarseSearch CT range to per-mode windows from
  `bayes.modes[]`. IMX179 (rear) runs here.

  Both impls share the downstream pipe: WB gains apply in the
  demosaic shader (zero silicon delay); CCM is LERP'd between
  calibrated brackets in `colorCorrection.Set[]` and lands in
  the shared `mCcmQ10` buffer the shader reads. See
  [project memory `project_awb_design.md`] and
  [docs/awb-bayes.md] for the migration plan + remaining
  calibration work.
- **AF** is a CDAF state machine in `AutoFocusController`:
  `Idle → Coarse1 → [Coarse2] → Fine → Settle`, parabolic peak
  interpolation, scene-change retrigger on continuous AF.
  Tap-to-focus parses `ANDROID_CONTROL_AF_REGIONS` (max 1 region
  per static metadata's `CONTROL_MAX_REGIONS = {0, 0, 1}`),
  expanded to 5×5-patch minimum. The same `FocusRoi` flows into
  `StatsWorker::Job` so the NEON encoder restricts Sobel /
  greenSq computation to the same window the state machine sums,
  and into `IpaFrameMeta` so AE meters the same region. Across a
  sweep AF holds `Ipa::setAeLock(true)` so AE doesn't chase
  brightness mid-scan.

## Configuration knobs

Compile-time, defined in `Android.mk`:

- `V4L2DEVICE_BUF_COUNT` — V4L2 capture ring size (default 8).

Runtime, via setprop (parsed in `hal/Camera.cpp`):

- `persist.camera.soft_isp` — `0` / `1`. `1` (default) enables the
  Vulkan ISP and the 3A controllers. `0` disables 3A and leaves
  exposure / gain under whatever the sensor kernel driver defaults
  to; kept as a fallback for cold bring-up but not exercised on
  the production path.

## Per-module tuning files

`/vendor/etc/camera/tuning/<lower(sensor)>_<lower(integrator)>.json`,
converted from the stock NVIDIA `.isp` overrides via
`tools/isp_to_json.py`. The JSON splits keys into:

- `active` — paths with a live HAL consumer today: `af.*` (VCM),
  `colorCorrection.Set[].ccMatrix` + `.cct` + `.wbGain`
  (multi-CCT CCM + neutral priors), `opticalBlack.*`, `ae.MeanAlg`
  (setpoint / damping / ratio clamps / tolerance), `awb.v4.*`
  (U→CCT polynomial, neutral / dark thresholds, smoothing).
- `reserved` — preserved 1:1 from the stock tuning (noise
  reduction, lens shading, tone curves, sharpness filters, full
  AE VFRTable, flicker detection, full AWB LUTs). Promoted to
  `active` as the corresponding HAL stage ships.
- `module` — per-module datasheet values not in the `.isp`
  (physical size, focal length, min focus distance, sensor
  orientation, bayer pattern). Merged in at conversion time from
  `tuning/_module_<sensor>_<integrator>.json`.

HAL-specific knobs that don't have a counterpart in NVIDIA's `.isp`
schema live under an `active.hal_overrides` sub-section *inside*
the same per-module JSON. `tools/isp_to_json.py` carves this section
out as preserve-verbatim across `.isp` regenerations. Currently
populated with `ae.close_speed_zone` (per-sensor width of the AE
LPF's faster-pole zone — see the AE summary above; IMX179 = 0
because the boost resonates with per-frame variability on rear cam
that hasn't been root-caused, OV5693 = 0.2). AF state-machine knobs
(`step_coarse`, `step_fine`, `contrast_ratio`, etc.) live directly
under `active.af` of the same JSON — there is no separate
`*_overrides.json` file and no deep-merge logic in
`SensorTuning::load`.

`SensorTuning` (`isp/sensor/`) loads at `Camera` construction.
`!isLoaded()` falls back to compile-time defaults so the HAL stays
alive even with the tuning dir missing — but the production path
expects the tuning installed.
