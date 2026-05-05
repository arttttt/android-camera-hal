# Roadmap

Prioritised list of improvements. Effort estimates are rough
(S = under a day, M = a few days, L = a week or more). Not a commitment
— a menu.

## Done

### Tier 1 — Camera3 compliance gaps (S–M)

Echoed per-frame controls back in result metadata; wired
`notifyError()` on every early-return path; honest
`PIPELINE_MAX_DEPTH` / `PARTIAL_RESULT_COUNT`; per-mode
`min_frame_duration` queried from `VIDIOC_ENUM_FRAMEINTERVALS`;
minimal AE/AWB state reporting (INACTIVE / CONVERGED / LOCKED) driven
by request mode + AF-sweep lock.

The two Tier 1 items that touched static characteristics —
`AVAILABLE_{REQUEST,RESULT,CHARACTERISTICS}_KEYS` and sensor
calibration keys — were folded into the `CameraStaticMetadata`
extraction. They are still outstanding as **Tier 1.2** below.

### Tier 1.1 — monolith splits (M)

Every big translation unit was audited against the project rules
(one object per file; SRP; sub-packages over flat dumps) and split.

- **`hal/Camera.cpp`**: 1400 → ~636 LOC. Behaviour ripped out into
  sub-packages under `hal/`:
  - `hal/3a/` — `AutoFocusController`, `AutoExposureController`, and
    the `Awb` interface with `GrayWorldAwbController` +
    `BayesianAwbController` impls behind `AwbFactory` (the three
    pure-return-style 3A surfaces; `Ipa3A` coordinates them).
  - `hal/metadata/` — `CameraStaticMetadata`, `RequestTemplateBuilder`,
    `ResultMetadataBuilder`.
  - `hal/jpeg/` — `JpegEncoder`.
  - `hal/pipeline/` — `StreamConfig`, `BufferProcessor`.
  `Camera` now dispatches into these per-frame and is mostly the
  Camera3 ops table + setup glue.

- **`isp/vulkan/VulkanIspPipeline.cpp`**: split into
  `isp/vulkan/runtime/` (`VulkanDeviceState`, loader/), `isp/vulkan/io/`
  (`VulkanInputRing`, `VulkanGrallocCache`), `isp/vulkan/shaders/`
  (GLSL headers for demosaic + blit), and `isp/sensor/IspCalibration`
  for per-sensor CCM tables. `IspParams` moved to `isp/IspParams.{h,cpp}`.

- **`image/ImageConverter.cpp`**: CPU Bayer / WB / CCM / gamma paths
  deleted during this tier; remaining YUV helpers removed in Tier 1.5.

- **`v4l2/Resolution.h`**: lifted `V4l2Device::Resolution` to a
  standalone header — struct is used across V4L2 / streams / metadata.

- **`util/DbgUtils.h`**: split into `AutoLogCall.h`, `FpsCounter.h`,
  `Benchmark.h`. `DbgUtils.h` stays as a thin facade including all
  three.

### Tier 1.5 — zero-copy everything, CPU fallback purge (M)

The HAL now has a single data path: Bayer → Vulkan ISP → gralloc (for
RGBA streams) or → CPU-mapped VkBuffer → libjpeg (for BLOB). No CPU
demosaic, no libyuv, no packed-YUV support.

- **Shader crop + scale** (push-constant `BlitPC`): the blit fragment
  samples `mScratchImg` via `sampler2D` with hardware bilinear.
  Identity path (`cropW == outW && cropH == outH`) uses `texelFetch`
  for zero overhead; scaling path uses the sampler's texture cache.
  `processToGralloc` grows `srcW / srcH` + `dstW / dstH` + `CropRect`;
  `zcEligible` drops `needZoom` and the resolution-match guard.

- **Per-stream GPU write**: removed the `!rgbaBuffer` short-circuit in
  `BufferProcessor::tryZeroCopy`. Each RGBA output runs its own GPU
  blit; the AF `SW_READ` lock fires once per frame and only on the
  first eligible output.

- **Zero-copy JPEG**: new `IspPipeline::processToCpu` returns a
  CPU-mapped pointer from the internal `mOutBuf`; libjpeg reads from
  it directly (no `mRgbaTemp` scratch). Orientation ships via an EXIF
  APP1 marker instead of `libyuv::ARGBRotate`, so no pixel rotation
  in the HAL. (Open Camera still sends `ANDROID_JPEG_ORIENTATION=0`
  when the user's device rotation sensor is broken — not a HAL bug;
  Libre Camera works on the same device.)

- **Deletions**: `ImageConverter`, `Yuv422UyvyToJpegEncoder`,
  `util/Workers`, `HwIspPipeline`, `IspPipeline::process`,
  `IspPipeline::processSync`, `Camera::mRgbaTemp`,
  `FrameContext::rgbaScratch`, the post-`SW_WRITE_OFTEN`-lock switch
  in `BufferProcessor::processOne`.

- **Binary size**: ~206 KB → ~88 KB.
- **Throughput**: ~18 fps → ~20 fps (bigger gains require Tier 2 / 3,
  `dqbuf` dominates the frame budget).

### Tier 2 — drain-to-latest + legacy define cleanup (S)

- `V4l2Device::readLock()` now drains the V4L2 done-queue to the newest
  frame before returning: the first DQBUF is blocking (`poll()`-gated),
  subsequent DQBUFs are non-blocking and each success requeues the
  previous slot. Worst-case preview staleness drops from
  `buf_count × frame_time` to `1 × frame_time` regardless of pipeline
  spikes. See [latency-and-buffers.md](latency-and-buffers.md) fix #1.
- The `V4L2DEVICE_USE_POLL` and `V4L2DEVICE_FPS_LIMIT` build-time knobs
  from the Antmicro import were removed as dead flexibility —
  `O_NONBLOCK` + `poll()` are unconditional now.

### Tier 2 — YUV_420_888 output (M)

- `IspPipeline::processToYuv420` produces NV12 in a host-mapped
  `VkBuffer` via a dedicated `VulkanYuvEncoder` (compute shader
  `RgbaToNv12`) composed into the pipeline's command buffer.
- `BufferProcessor::processYuvOutput` repacks NV12 into whatever
  chroma layout `GraphicBufferMapper::lockYCbCr` returns — NV12
  (direct plane copies) and I420 / YV12 (`libyuv::NV12ToI420`) are
  supported; NV21 returns `NO_INIT` for now. See
  [open-questions.md](open-questions.md) for the full-coverage path.
- `StreamConfig::normalize` resolves `IMPLEMENTATION_DEFINED` by
  usage: `HW_VIDEO_ENCODER` → YUV_420_888, everything else → RGBA_8888.
- `CameraStaticMetadata` advertises `YCbCr_420_888` output configs
  at every sensor-supported resolution — CameraX ImageAnalysis /
  MLKit / video encoder streams can now pick it.
- BT.601 limited-range is hardcoded in the shader. Proper colour-space
  metadata + BT.709 / full-range support is not scheduled — see
  open-questions.

### Tier 2 — JSON tuning files per module (M)

Stock NVIDIA `.isp` overrides from the Mi Pad 1 vendor blob converted
into HAL-owned JSON under `/vendor/etc/camera/tuning/` (Treble path).
Every hardcoded sensor constant that the HAL has a live consumer for
now reads from the JSON; remaining keys are preserved verbatim in a
`reserved` section so future ISP stages (NR, LSC, tone curves,
sharpness, full AE VFR, AWB CCT LUT) land without another conversion.

- `tools/isp_to_json.py` — one-shot converter. Parses NVIDIA's
  `namespace.path[i].sub = value;` syntax (including multi-line
  tuples with missing `};` and split LHS / `=value;` lines).
  Splits output into `active` (paths consumed by current HAL) and
  `reserved` (everything else). Hybrid containers (`Chroma.Enable`
  + `Chroma[0].Gain`) survive via dict-with-stringified-numeric-keys,
  normalised back to JSON arrays where pure 0..N-1.
- `isp/sensor/SensorTuning.{h,cpp}` — runtime loader (jsoncpp static).
  Filename derived by convention: `<lower(sensor)>_<lower(integrator)>.json`.
  On file missing / malformed / schema mismatch: `!isLoaded()`,
  consumers fall back to compile-time defaults.
- `tuning/imx179_primax.json` + `tuning/ov5693_sunny.json` (~150 KB
  each, faithful 1:1 conversion). Installed via `BUILD_PREBUILT`.

Consumers wired:
- `AutoFocusController` — VCM infinity / macro / offsets / settle-time
  from `active.af.*`. Calibrated infinity position becomes
  `af.inf + af.inf_offset` (NVIDIA convention).
- `IspCalibration` deleted — CCM now picked from
  `active.colorCorrection.Set[]` via `SensorTuning::ccmForCctQ10()`,
  nearest-CCT match at a pinned 5000 K (closest daylight point
  available on both profiles; D65 isn't there). CCT-driven + `wbGain`
  selection deferred to Tier 3 AWB — flagged in
  [open-questions.md](open-questions.md).
- `DemosaicCompute` shader — optical-black subtract + dynamic-range
  rescaling using `active.opticalBlack.manualBias*`. Visible
  improvement in shadow purity (no more warm bias floor).
- `CameraStaticMetadata` — `ANDROID_SENSOR_INFO_PHYSICAL_SIZE`,
  `ANDROID_LENS_INFO_AVAILABLE_FOCAL_LENGTHS`, and
  `ANDROID_LENS_INFO_MINIMUM_FOCUS_DISTANCE` from `module.*` (not
  from the `.isp` — these are per-module datasheet facts merged in
  at conversion time via `tuning/_module_*.json`).

Not yet consumed (still in JSON, not in HAL):
- `active.mwbCCT.*` — manual-AWB preset reference points; blocked
  on `ANDROID_CONTROL_AWB_MODE` handling past AUTO/OFF.
- `active.colorCorrection.srgbMatrix` — D50 sRGB transform; would
  apply after the illuminant-specific CCM in a correctly implemented
  two-stage colour pipeline. Tier 3.
- `active.colorCorrection.Set[].wbGain` — per-CCT WB priors, see
  open-questions CCM section.
- Everything under `reserved.*` (NR, LSC, tone curves, sharpness,
  flicker, full AE / AWB LUTs).

## Tier 1.2 — remaining Camera3 compliance (S)

Absorbed into the `CameraStaticMetadata` home but not yet implemented.
Each is a short edit inside that module.

- **`AVAILABLE_{REQUEST,RESULT,CHARACTERISTICS}_KEYS` arrays** — needed
  by CameraX feature-availability probes. Without them some clients
  fall back to safe defaults.

- **Sensor calibration keys**: `BLACK_LEVEL_PATTERN`, `WHITE_LEVEL`,
  `COLOR_FILTER_ARRANGEMENT`, `COLOR_TRANSFORM_{1,2}`,
  `FORWARD_MATRIX_{1,2}`, `CALIBRATION_TRANSFORM_{1,2}`,
  `REFERENCE_ILLUMINANT{1,2}`, `NOISE_PROFILE`. Per-sensor values from
  `SensorTuning` (now live) — a short edit in `CameraStaticMetadata`
  to emit them from `module.*` + `active.colorCorrection.*`. Unlocks
  DNG output.

## Tier 3 — asynchronous event-driven pipeline (L)

Full architecture is specified in
[**tier3_architecture.md**](tier3_architecture.md). Summary below.

Seven threads per camera (binder + RequestThread + CaptureThread +
PipelineThread + StatsWorker + on-demand PostprocWorker; ResultThread
is rolled into PipelineThread for single-stream preview). All
cross-thread comms via bounded queues + eventfd + sync_fd. Every GPU
wait is a fence fd registered in `poll()`; `vkWaitForFences` only on
`flush()` / `close()` / hang-recovery.

Expected from the original plan: single-stream ~20 fps → ~28–30 fps
on 1080p; multi-stream preview+video ~13 fps → ~28 fps. Foundation
for ZSL, reprocess, real 3A, and Tier 3.5 produce-once without
further rewrite.

**Done**

1. **PR 1** — Threading primitives: `ThreadBase`, `EventQueue<T>`,
   `Signal<T>`, `UniqueFd`, `EventFd`. `ThreadBase::start` drains
   the stop eventfd so restart-after-stop actually runs.
2. **PR 2** — `RequestQueue` + `RequestThread` + `Pipeline` +
   `PipelineStage` + `PipelineContext` + `InFlightTracker` + the
   five concrete stages (ApplySettings, ShutterNotify, Capture,
   DemosaicBlit, ResultDispatch). `processCaptureRequest` returns
   in < 1 ms; the cache (`mLastRequestSettings`) is updated
   synchronously on the binder thread to avoid a racy BAD_VALUE
   against follow-up requests with settings=NULL.
3. **PR 3** — `BayerSource` / `V4l2Source` / `V4l2CaptureThread`;
   drain-to-latest migrated out of `V4l2Device::readLock` into the
   capture thread. Lifecycle rework landed at the same time:
   infrastructure (ISP / 3A / BufferProcessor / BayerSource /
   pipeline / worker thread) is built once in `openDevice` and
   survives `close → reopen`; `stopWorkers()` is the single
   quiesce primitive (stops threads, drains GPU) and `closeDevice`
   additionally clears per-session state (`mLastRequestSettings`,
   AF state, `VulkanGrallocCache`). `V4l2Device::setStreaming(true)`
   re-queues every slot before STREAMON; the kernel returns all
   buffers to USERSPACE on STREAMOFF so the next session's STREAMON
   needs a fresh QBUF of every slot.
4. **PR 4** — `PipelineThread` + Vulkan `external_fence_fd` in
   `poll()`. `waitForPreviousFrame` dropped from the hot path; GPU
   submits ring of depth 4 hand their sync_fds into a fence-fd poll
   set, so the `wait` segment of the PERF log collapses from ~55 ms
   to near zero. Alongside the thread split: async result dispatch,
   `errorCompletePendingRequests` on session boundaries to drain
   orphaned ctxs that `stopWorkers` parks upstream of PipelineThread.
   Demosaic shader perf pass (first-review fixes: float-recip
   `readPixel`, cooperative shared Bayer tile, 16×16 WG, sRGB LUT,
   Blit uv precompute, NV12 stride) landed on the same tier and
   dropped the per-frame demosaic cost from 47 ms to ~3.4 ms at 1080p.
5. **PR 5 — rolled into PR 4.** The planned `ResultThread` split was
   unnecessary: single-stream preview already ran framework callbacks
   off the binder thread via `PipelineThread`, and a dedicated
   `ResultThread` would only have helped with multi-stream JPEG /
   video interleaving — which waits on the PR 7 JpegWorker split
   anyway.
6. **PR 6 — Ipa + DelayedControls + stats plumbing.** `Ipa` interface
   and `StubIpa` (empty-batch wrapper) live under `hal/ipa/`;
   `DelayedControls` seeded from `SensorConfig::controlDelay` (2
   frames for both IMX179 and OV5693 as a libcamera-convention
   default — verify empirically when a real loop drives it);
   `StatsProcessStage` sits between fence reap and result dispatch
   on `PipelineThread` and wires the producer path. `closeDevice`
   resets `Ipa` and `DelayedControls` alongside existing session
   cleanups. `Ipa3A` (real AE / AWB / AF) is a follow-up PR; what
   this one lands is the shape.
7. **NEON stats experiment (post-PR-6).** Original design had the
   statistics producer as a GPU compute shader (histogram + patch
   means + Tenengrad into a host-mapped `VkBuffer`); that shipped
   transiently but cost GPU time on the hot path and forced a
   temporal-subsample throttle (stats only every other frame) to
   fit the frame budget. Replaced by a dedicated `StatsWorker`
   thread running a NEON-vectorised kernel over the raw Bayer slot
   (`NeonStatsEncoder`): CPU compute overlaps the Vulkan submit
   instead of serialising with it, GPU stays demosaic + blit only.
   One IpaStats cycle spans `phaseCount` submits (default 2),
   spreading the ~8 ms NEON pass over two sensor periods so peak
   CPU per frame stays below ~4 ms; `phaseCount = 1` falls back to
   one-shot compute via a single constant flip. Raw-Bayer semantics
   for rgbMean / lumaHist match the libcamera IPU3 / rkisp1
   convention — `Ipa3A`'s AE loop (next entry) consumes these
   directly in the pre-WB / pre-CCM domain. Binary shrank ~8 KB
   when the Vulkan stats encoder + shader + base-class virtuals
   were removed.
8. **Ipa3A AE + ApplySettings AE-mode branch (post-PR-6.5).**
   `Ipa3A` owns the AE loop. Originally shipped as a P-controller
   with EMA damping over the green-channel mean-luma histogram,
   setpoint 0.35 (pre-gamma proxy for 18 % mid grey), hardcoded
   ratio clamp ∈ [0.5, 2.0] and exposure ceiling `kMaxExposureUs =
   200 ms`. Refactored substantially since:
   setpoint moved to `pow(MeanAlg.target/255, 2.2)` from tuning;
   stats source switched from histogram to focus-ROI patch-grid
   (`rgbMean[*][*][1]`); ratio clamps and damping promoted to
   tuning's `MaxFstopDelta` / `ConvergeSpeed`; exposure ceiling
   replaced by `maxExposureUsDefault × gainMax` from
   driver-queried envelope; AE_LOCK + EV-comp + AF-coordinated
   lock added; cascade EMA on the multiplier ripped out and
   replaced with absolute-EV-space target + single-pole LPF +
   2 % dead-band + RPi-style top-2% IQM highlight constraint
   (commits `7c3bc7e`, `99a7e92`, `b061a76`, `997f7c6`, `fabed2f`,
   `2947b17`, 2026-05-01). Current shape lives in
   `tier3_architecture.md#basicipa-ae-loop-landed` — read that for
   the up-to-date description.

   `ApplySettingsStage` branches on `ANDROID_CONTROL_AE_MODE`:
   `OFF` → manual (parse request, write V4L2, publish into
   `DelayedControls` for result-metadata consistency); non-`OFF`
   → auto (read `DelayedControls::pendingWrite(frameNumber)` —
   the new read API — and push through `applyBatch` as one
   `VIDIOC_S_EXT_CTRLS`, falling back to the manual path if the
   ring is cold). `DelayedControls` is now the single source of
   truth for applied exposure / gain across both modes, guarded
   by a mutex (producers on PipelineThread + binder, consumers on
   RequestThread + result builder). `StatsProcessStage` skips its
   `DelayedControls::push` when AE is OFF so framework authority
   is never fought by the IPA. Dark-scene policy is FPS-priority:
   AE clamps at the sensor's `maxExposureUsDefault` inside the
   default `frame_length` rather than extending integration at the
   cost of frame rate.
9. **Ipa3A AWB + AF (post-PR-6.6 / 6.7 / 6.8).** Gray-world AWB
   over `rgbMean[16][16][3]` with CCT estimation from ln(G/B) and
   per-CCT CCM LERP across the tuning's `colorCorrection.Set[]`;
   shader gets WB gains every frame (zero silicon delay) and a
   shared Q10 CCM buffer. AF moved off the gralloc-RGBA Laplacian
   onto the NEON-computed `IpaStats::focusMetric` —
   `Σ(Gx²+Gy²)/Σ I²` per patch, exposure-invariant. Coarse-fine
   state machine (`Idle → Coarse1 → [Coarse2] → Fine → Settle`)
   with parabolic peak interpolation over a vector-backed scan
   history; continuous AF retriggers via a multi-channel snapshot
   (focusMetric + R / G / B mean over the centre 8×8 patches).
   AE coordination through `Ipa::isAeConverged()` and
   `Ipa::setAeLock(bool)` — the lock republishes the converged
   exposure / gain every frame so DelayedControls keeps the sensor
   put across a sweep. NEON kernel skips Sobel / greenSq outside
   the focus ROI (the AF centre 8×8) to drop the stats cycle from
   ~17 ms to ~12 ms on 720p; `IpaStats::sharpness` was a dead
   field and got dropped along the way. Continuous retrigger on
   target-distance changes is imperfect — accepted as
   good-enough; tracked in
   [bugs.md](bugs.md) and the AF-references memory.
10. **Housekeeping — `V4L2DEVICE_OPEN_ONCE` removed.** The flag is
    gone from `Android.mk`, `README`, and `docs/architecture.md`;
    the V4l2Device ctor no longer auto-connects, `disconnect()`
    always runs `cleanup()` so the fd closes at end of session,
    and `cleanup()` invalidates the cached `mFormat` so the next
    `connect()` re-runs `S_FMT` + `REQBUFS` against whichever
    memory type the next session ends up using. Pre-open
    enumeration still works because `availableResolutions()` /
    `minFrameDurationNs()` open a temporary fd when `mFd < 0`.
    Camera switching (front ↔ back close / reopen) verified on
    device.
11. **PR 7 — Produce-once + JpegWorker + ResultThread split.**
    `IspPipeline` grows `beginFrame` / `blitToGralloc` / `blitToYuv`
    / `blitToJpegCpu` / `endFrame`; the demosaic runs once per frame
    into `mScratchImg` and each output blits / encodes / copies from
    that scratch in a single Vulkan submit. Framework acquire_fence
    sync_fds are imported as binary `VkSemaphore`s
    (`VK_KHR_external_semaphore_fd`) so the recording thread never
    blocks on framework backpressure; per-output `release_fence`s
    fan out through `vkQueueSignalReleaseImageANDROID`. `PostProcessor`
    abstracts BLOB encoding; `JpegEncoder` is the only impl today
    (libjpeg + EXIF Orientation marker). The dispatch side splits in
    two: `ResultThread` owns `ResultDispatchStage` + Bayer flush +
    `InFlightTracker::removeBySequence`, and `JpegWorker` runs libjpeg
    on its own thread. Per-ctx `std::atomic<int> jpegPending` gates
    `ResultThread`'s dispatch so Camera3's monotonic frame_number
    ordering is preserved while sensor frames keep flowing on
    `PipelineThread` during JPEG encode. Legacy `processToGralloc` /
    `processToYuv420` / `processToCpu` removed alongside their
    helpers (`recordGrallocBlit`, `submitWithReleaseFence`,
    `recordDemosaicAndYuvEncode`, `recordAndSubmit`, `mOutBuf`).

Deferred but slot-reserved from PR 2: `Request::inputBuffer` for
ZSL / reprocess; ZSL ring buffer and reprocess wiring happen in
Tier 4 with no queue-type churn.

### Housekeeping — drop `V4L2DEVICE_OPEN_ONCE` entirely — done

Item 10 above. Captured here for the historical note on what the
flag was and why it had to go.

### Camera2 / HAL3.4 compliance pass — done

Brought `device_version` from 3.0 up to 3.4 incrementally and closed
the Camera2 contract gaps that were keeping app paths broken:

- Static metadata: `AVAILABLE_REQUEST/RESULT/CHARACTERISTICS_KEYS`,
  `REQUEST_AVAILABLE_CAPABILITIES = [BACKWARD_COMPATIBLE]`,
  `MAX_NUM_OUTPUT_STREAMS` (or the API-26+ split), `SYNC_MAX_LATENCY`,
  per-stage `EDGE / HOT_PIXEL / NOISE_REDUCTION / SHADING /
  TONEMAP / COLOR_CORRECTION_ABERRATION /
  LENS_INFO_AVAILABLE_OPTICAL_STABILIZATION / SENSOR_AVAILABLE_TEST_PATTERN /
  STATISTICS_INFO_AVAILABLE_FACE_DETECT / HOT_PIXEL_MAP /
  LENS_SHADING_MAP` AVAILABLE arrays, `LENS_INFO_HYPERFOCAL_DISTANCE`,
  `LENS_INFO_FOCUS_DISTANCE_CALIBRATION`, `LENS_INFO_AVAILABLE_APERTURES /
  FILTER_DENSITIES`, `SENSOR_INFO_TIMESTAMP_SOURCE`,
  `SENSOR_INFO_COLOR_FILTER_ARRANGEMENT`, `CONTROL_AVAILABLE_MODES`.
- Honest `REQUEST_PIPELINE_MAX_DEPTH = 4`.
- HAL3.2 ABI: NULL'd deprecated `register_stream_buffers` and
  `get_metadata_vendor_tag_ops`; emit `partial_result = 1`;
  preserve consumer gralloc usage flags in `StreamConfig::normalize`
  (only add SW_WRITE_OFTEN for BLOB outputs that libjpeg fills
  CPU-side).
- HAL3.3 ABI: validate
  `camera3_stream_configuration_t::operation_mode` (NORMAL only),
  `camera3_stream_t::rotation` (0 only), reject DEPTH `data_space`.
- AE contract: report `ANDROID_CONTROL_AE_STATE` from Ipa3A
  convergence + AE_LOCK; honour `AE_EXPOSURE_COMPENSATION` on the
  auto path and as an additive offset through AE_LOCK with EMA
  smoothing to avoid the rolling-shutter readout split on big
  steps.
- AF contract: `ANDROID_CONTROL_MAX_REGIONS = {0, 0, 1}`,
  `AF_REGIONS` parsed in `AutoFocusController::onSettings` into a
  patch-grid `FocusRoi` (5×5-patch minimum around tap centre,
  thread-safe getter), wired through `StatsWorker::Job` so the
  NEON encoder restricts Sobel / greenSq to the same window the
  state machine sums.
- AutoFocusController hardcode-to-tuning: VCM range / settle
  frames / step / contrast / retrigger ratios all from
  `SensorTuning`, no compile-time fallbacks; `mVcmPerDiopter`
  derived from calibrated VCM range and module's
  `min_focus_distance_diopters`; `clampVcm` bounds to
  `[mVcmInfinity, mVcmMacroEnd]` so manual-focus sliders past
  MIN_FOCUS_DISTANCE rest at macro instead of driving the
  actuator into a hardware stop.
- `module.sensor_orientation_degrees` per-module in tuning JSON
  instead of a `(facing == FRONT) ? 270 : 90` ternary.

After this, the HAL claims 3.4 and apps that gate on hardware
level / device version see the freshest contract from us; no new
features behind the bumps but the surface that's there matches
what the framework expects.

### Output cap — done

`hal/pipeline/OutputResolutionCap.h` filters advertised stream
configurations to **16:9 ≤ 1920×1080** (`acceptsAdvertised`) and
caps the size of any stream the framework asks `configureStreams`
to allocate (`acceptsSize`, accepts any aspect up to the same
ceiling so MediaRecorder thumbnail / face-detect callbacks at
non-16:9 sizes pass through). Above 1080p the SW ISP — Vulkan
demosaic + blit + libjpeg encode — becomes FPS-bound, and the
post-demosaic scratch ring is sized for a single fixed
1920×1080 RGBA slot per `SLOT_COUNT`. 4:3 photo modes will come
back when the kernel sensor-side cropping profiles grow proper
4:3 support; that side will pick those up.

### Focus-tied AE — done

AE meters from `IpaStats::rgbMean[*][*][1]` (G channel, raw
pre-WB / pre-CCM domain) inside the active focus ROI. Two
revisions:

1. **Centre-weighted form** (commit `dc08f08`) — initial replacement
   for the pre-existing `lumaHist`-based mean. The histogram path
   produced an AE setpoint that didn't follow tap-to-focus and
   spent NEON inner-loop work on a field nobody read.
2. **Spot meter inside ROI** (commit `ad9903e`) — the weighted
   form's 2x-vs-1x bias was too subtle on a real device, so
   `meanLumaInRoi` switched to a plain mean over patches inside
   the ROI only. Default ROI = centre 8×8 (matches the
   `IpaStats::FOCUS_ROI_*` constants the rest of the project
   already uses), so without a tap AE meters a 64-patch centre
   spot. With a tap AE meters precisely the user's chosen subject
   at 5×5+ patches.

`StatsProcessStage` plumbs `AutoFocusController::currentFocusRoi()`
into `IpaFrameMeta::focusRoi*` so AE / AWB-gate / AF all share one
rectangle source-of-truth.

`AutoFocusController::onSettings` widened the `isFullFrame` check
to a 1-pixel margin (commit `aa1c8ea`) so `(0, 0, sw-1, sh-1)`-style
"no specific subject" hints from apps don't get parsed as a real
tap — without that the ROI was a full 16×16 grid and the spot
meter degenerated back to a uniform mean.

`lumaHist` field and the per-pixel scatter in NeonStatsEncoder
were dropped (commit `ad34c55`) once nothing reads them.

### AWB confidence gate + prior-relax — done

`Ipa3A` AWB raised `awbMinValidPatches` from 32 → 96 (37.5 % of
the 16×16 grid) and added a symmetric pull-to-prior on gate
failure (commits `9806ce9`, `d1173a1`). Below the gate `lastWb`
EMA-relaxes back to `wbGainPrior` (per-CCT calibrated daylight
neutral) at the same `awbDamping` the forward path uses.

Why: gray-world over a small biased subset (dim scene, a handful
of valid patches that all sit on a non-neutral lit object —
laptop screen, fluorescent bulb, lampshade) used to land
incorrect WB gains and freeze them in place when the scene fell
back below the gate. The gate + symmetric relax means dim scenes
fall back to "looks daylight-cool" instead of cyan / blue-green.

The cast was originally diagnosed in `bugs.md` as gain-dependent
black-level drift; live diagnostics ruled that out (cast persists
at sensor unity gain, no gain-dependent OB calibration in any of
the `.isp` revisions or the kernel V4L2 driver). Manual dark-frame
calibration would unlock a more-correct OB fix as a deferred
follow-up. See the project memory `project_awb_design.md` for
the full reasoning.

### Bayesian AWB — shipped on IMX179 (partial calibration)

After live diagnostics confirmed the cyan-cast on real (non-dim)
scenes was a structural gray-world failure on colour-dominant
average — not an OB / clamp / prior-shape bug — the AWB path was
rebuilt around an RPi-style Bayesian estimator. Migration plan in
`docs/awb-bayes.md` (Steps 1-9 done, Step 10 partial, Step 11
shipped for IMX179, Step 12 deferred).

Pipeline lands in `hal/3a/BayesianAwbController` behind a common
`Awb` interface; `AwbFactory::createAwb` picks gray-world or
bayes per-sensor from `active.awb.algorithm` + a populated
`bayes` block. Per tick:

- coarseSearch over `bayes.ctCurveR / ctCurveB` PWLs, log-stepped
  across the curves' calibrated CT domain. Cost per candidate t:
  `Σ_zones min(δ², deltaLimit) + biasWeight·min(δ_bias², deltaLimit)
  − prior(t | luxIndex)` with zones = (R/G, B/G) per patch and
  `(gainR, gainB) = (1/ctR(t), 1/ctB(t))`.
- fineSearch refines off-curve along the perpendicular axis in
  (R/G, B/G) space, capped by `transversePos / transverseNeg`,
  for magenta ↔ green correction the on-curve search can't
  express.
- Output gains IIR-smoothed at `damping` after `startupFrames`
  cold-start snap.
- Manual AWB presets (INCANDESCENT / DAYLIGHT / …) supported by
  clipping the coarseSearch CT range to per-mode windows from
  `bayes.modes[]`. The matching modes are also advertised in
  `ANDROID_CONTROL_AWB_AVAILABLE_MODES` only when the sensor
  actually has bayes calibration — gray-world keeps AUTO / OFF.

IMX179 ships bayes with calibration data derived from the
existing `awb.v4.FusionLights` raw `[R, GR, GB, B]` measurements
(post-OB) — eight inner CT points across ~3500-5350 K, plus
extrapolated 2700 K and 7000 K endpoints for INCANDESCENT /
SHADE preset coverage. OV5693 stays on gray-world for now;
proper grey-card calibration of both sensors (Step 10 full)
is the open follow-up.

Closes the `manual AWB presets` open-question item from
`open-questions.md`.

## Open

- **ZSL + reprocess** — slot reserved in `PipelineContext`
  (`Request::inputBuffer`, `MAX_NUM_INPUT_STREAMS = 0` placeholder
  in static metadata). Ring of N most-recent frames + reprocess
  wiring through Camera3's input stream contract. Earlier draft
  bumped `SLOT_COUNT` to 4 to back the ring; that blew the shared
  nvmap pool and was rolled back, so a future ZSL has to size its
  retention against the existing depth-2 ring or use a different
  storage strategy. Discretionary.
- **Gain-dependent optical-black calibration** — deferred more-
  correct fix for the IMX179 high-gain cast bugs.md entry. Needs
  a dark-frame calibration session on-device (capture covered-lens
  raw at gain 1×, 2×, 4×, 8×, 16×, 32×, 64×, fit per-channel
  pedestal curves, tabulate as a tuning addendum). Currently
  masked by the AWB confidence gate.
- **Live `bugs.md` items** — OV5693 slow ISO pulsation, vertical
  seam at low exposures, V4L2 fd reopen perf. None blocking apps.
