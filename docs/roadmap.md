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
  - `hal/3a/` — `AutoFocusController`, `ExposureControl`.
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
   cleanups. `BasicIpa` (real AE / AWB / AF) is a follow-up PR; what
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
   convention — `BasicIpa`'s AE loop (next entry) consumes these
   directly in the pre-WB / pre-CCM domain. Binary shrank ~8 KB
   when the Vulkan stats encoder + shader + base-class virtuals
   were removed.
8. **BasicIpa AE + ApplySettings AE-mode branch (post-PR-6.5).**
   `BasicIpa` now owns the AE loop: P-controller with EMA damping
   over the green-channel mean-luma histogram, setpoint 0.35
   (pre-gamma proxy for 18 % mid grey), ratio clamp ∈ [0.5, 2.0].
   The (exposure, gain) split goes through
   `SensorConfig::splitExposureGain` which prefers exposure up to
   `kMaxExposureUs = 200 ms` before pushing into analog gain.
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
   AE clamps at `kMaxExposureUs` inside the default
   `frame_length` rather than extending integration at the cost
   of frame rate. AWB / AF are still `StubIpa`-equivalent
   (next item).

**Pending**

9. **Produce-once refactor** (`IspPipeline::beginFrame` / `blitTo*` /
   `endFrame`) + `PostProcessor` / `JpegWorker`. **Multi-stream FPS
   win** (preview+video 13 → ~28). This is the PR 7 slot.
10. **BasicIpa AWB + AF** — gray-world AWB over
    `rgbMean[16][16][3]`, and AF integration feeding
    `sharpness[16][16]` to `AutoFocusController` instead of the
    current gralloc `SW_READ_OFTEN` lock. No new threading; both
    extend `BasicIpa::processStats`.

Deferred but slot-reserved from PR 2: `Request::inputBuffer` for
ZSL / reprocess; ZSL ring buffer and reprocess wiring happen in
Tier 4 with no queue-type churn.

### Housekeeping — drop `V4L2DEVICE_OPEN_ONCE` entirely (S)

After PR 8 lands, remove the remaining `V4L2DEVICE_OPEN_ONCE`
fast-paths from `v4l2/V4l2Device.cpp` (ctor auto-`connect()`,
`disconnect()` guarded `cleanup()`) and from `Android.mk` /
`README` / `docs/architecture.md`. The flag dates from an earlier
attempt to avoid sensor reinit cost across close/reopen; the
STREAMOFF half was already dropped when the stale-frame-across-
sessions bug surfaced (mainline now does real STREAMOFF and
re-queues every slot in `setStreaming(true)`). The fd-lifetime
half kept an fd across `closeDevice`, which in turn forced the
Camera ctor's implicit-connect assumption for early
`staticCharacteristics()` queries. Clean-up plan:

- V4l2Device ctor: no auto-`connect()`.
- `disconnect()`: always `cleanup()`; fd closes at end of session.
- Camera ctor: call `mDev->connect()` explicitly so pre-open
  enumeration still works.
- Verify reopen path re-negotiates pixel format on fresh fd.
- Drop the `-DV4L2DEVICE_OPEN_ONCE` define and all its docs.

## Tier 4 — aspirational (L, or rewrite)

### Move sensor / ISP access to media-controller + v4l2-subdev

Today `/dev/video0` is the only node we touch (plus the focuser
subdev). Multi-stage ISP pipelines require addressing the subdevs
directly: select sensor modes via `VIDIOC_SUBDEV_S_FMT`, route
statistics through a separate capture node, etc.

Path to eventually adopting libcamera as the backing layer, or at
minimum to supporting Tegra variants with different topologies without
rewriting the HAL.

### Replace bespoke HAL with libcamera's Android shim

libcamera ships `src/android/` — a Camera3 HAL backed by libcamera.
If a Tegra pipeline handler existed upstream (none does today), we
could retire this HAL entirely. Writing that pipeline handler is
realistically a months-long project and ties us to libcamera's
release cadence; noted here for completeness rather than as an
imminent plan.

## Suggested sequencing

1. **Tier 1.2** — short compliance PRs inside `CameraStaticMetadata`.
2. **Tier 2** — done (drain-to-latest, `YUV_420_888` output, JSON
   tuning).
3. **Tier 3 PR 1-8** — done (threading primitives, RequestThread,
   CaptureThread, PipelineThread + fence-fd, IPA/DelayedControls
   plumbing, NEON stats worker, BasicIpa AE + ApplySettings
   AE-mode branch).
4. **Tier 3 produce-once** — `IspPipeline::beginFrame` + `blitTo*` +
   `endFrame`, PostProcessor + JpegWorker. Multi-stream FPS win.
5. **Tier 3 BasicIpa AWB + AF** — gray-world AWB, sharpness-grid
   AF (removes the last `SW_READ_OFTEN` lock).
6. **Housekeeping** — drop `V4L2DEVICE_OPEN_ONCE` fast paths.
7. **Tier 4** — discretionary.
