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

Six threads per camera; all cross-thread comms via bounded queues +
eventfd + sync_fd. Every GPU wait is a fence fd registered in `poll()`;
`vkWaitForFences` only on `flush()` / `close()` / hang-recovery.

Expected: single-stream ~20 fps → ~28–30 fps on 1080p; multi-stream
preview+video ~13 fps → ~28 fps. Foundation for ZSL, reprocess, real
3A, and Tier 3.5 produce-once without further rewrite.

Landed in eight PRs (each shippable + testable on its own; split
chosen so `git bisect` cleanly attributes FPS and correctness shifts):

1. **PR 1** — Threading primitives: `ThreadBase`, `EventQueue<T>`,
   `Signal<T>`, `UniqueFd`. Infra only, no consumers.
2. **PR 2** — `RequestQueue` + `RequestThread` wrapping the current
   synchronous path. `processCaptureRequest` returns in < 1 ms.
3. **PR 3** — `BayerSource` / `V4l2Source` / `CaptureThread`;
   drain-to-latest migrates out of `V4l2Device`.
4. **PR 4** — `DelayedControls` port (~280 LOC) with
   `SensorTuning.controlDelay`; truthful result metadata.
5. **PR 5** — `PipelineThread` + Vulkan `external_fence_fd` in
   `poll()`. `waitForPreviousFrame` deleted; GPU depth > 1. **FPS
   win lands here** (preview 20 → ~28–30).
6. **PR 6** — `ResultThread` split. Framework callbacks off
   `PipelineThread`; FIFO by single-thread construction.
7. **PR 7** — `Ipa` interface + GPU statistics compute shader;
   AE / AWB / AF driven by real stats, not gralloc-locked preview.
8. **PR 8** — Produce-once refactor
   (`IspPipeline::beginFrame` / `blitTo*` / `endFrame`) + `PostProcessor` /
   `JpegWorker`. **Multi-stream FPS win** (preview+video 13 → ~28).

Deferred but slot-reserved from PR 2: `Request::inputBuffer` for
ZSL / reprocess; ZSL ring buffer and reprocess wiring happen in
Tier 4 with no queue-type churn.

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
2. **Tier 2** — drain-to-latest (quick latency win), `YUV_420_888`
   output, JSON tuning (cleanup that makes Tier 3 easier).
3. **Tier 3** — branch: request-queue refactor → DelayedControls →
   IPA module split.
4. **Tier 3.5** — produce-once / sample-many ISP, after Tier 3 lands.
5. **Tier 4** — discretionary.
