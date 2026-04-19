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
  deleted (dead — Vulkan handles every Bayer request). Only YUV → RGBA /
  JPEG helpers remain; scheduled for full removal in Tier 1.5.

- **`v4l2/Resolution.h`**: lifted `V4l2Device::Resolution` to a
  standalone header — struct is used across V4L2 / streams / metadata.

- **`util/DbgUtils.h`**: split into `AutoLogCall.h`, `FpsCounter.h`,
  `Benchmark.h`. `DbgUtils.h` stays as a thin facade including all
  three.

## Tier 1.2 — remaining Camera3 compliance (S)

Absorbed into the `CameraStaticMetadata` home but not yet implemented.
Each is a short edit inside that module.

- **`AVAILABLE_{REQUEST,RESULT,CHARACTERISTICS}_KEYS` arrays** — needed
  by CameraX feature-availability probes. Without them some clients
  fall back to safe defaults.

- **Sensor calibration keys**: `BLACK_LEVEL_PATTERN`, `WHITE_LEVEL`,
  `COLOR_FILTER_ARRANGEMENT`, `COLOR_TRANSFORM_{1,2}`,
  `FORWARD_MATRIX_{1,2}`, `CALIBRATION_TRANSFORM_{1,2}`,
  `REFERENCE_ILLUMINANT{1,2}`, `NOISE_PROFILE`. Per-sensor values
  from `SensorConfig` / `IspCalibration` (or the JSON tuning file once
  Tier 2 lands). Unlocks DNG output.

## Tier 1.5 — eliminate CPU fallbacks, zero-copy everything (M)

Today zero-copy only triggers when **all** of these hold simultaneously:
RGBA_8888 output, stream size equals V4L2 capture size, no zoom, Bayer
input, first RGBA buffer of the request. Anything else falls back to a
CPU path that demosaics → reads back → runs libyuv → encodes — killing
the ~45 ms frame budget and re-introducing the 25 ms blocklinear detile
cost on the `SW_WRITE_OFTEN` gralloc lock.

Goal of this tier: **delete every CPU fallback** in
`BufferProcessor::processOne`. If a case cannot be serviced on the GPU,
we don't support it.

### 1. Zero-copy zoom

`needZoom=true` today → `libyuv::ARGBScale` after a full GPU readback.
Push the crop rect into the existing fragment-shader blit:

- Feed `(cropX, cropY, cropW, cropH)` as a push constant (or small UBO)
  to the shader that writes `mScratchImg` → gralloc.
- Shader samples `mScratchImg` at `ivec2(cropX + u*cropW, cropY + v*cropH)`.
- `imageLoad` is unfiltered — emulate bilinear with 4 loads + manual lerp,
  or make `mScratchImg` a sampled image (requires checking whether Tegra
  supports samplers on pitchlinear storage images).

### 2. Zero-copy cross-resolution

Same shader covers the `streamW != res.width || streamH != res.height`
case — `cropRect` degenerates to full-frame-at-output-scale. Drop the
resolution check from `zcEligible`.

### 3. Per-stream GPU write (no `rgbaBuffer` cache)

Current code short-circuits zero-copy when another output already got a
CPU RGBA render. With (1)+(2) each stream gets its own
`processToGralloc` call — remove the `!rgbaBuffer` clause and the
shared-buffer cache.

### 4. Zero-copy JPEG

Everything before `libjpeg` moves to GPU:

- Allocate a CPU-mappable dma-buf-backed `VkBuffer` (OPAQUE_FD export,
  same pattern as the input ring). Cache per sensor resolution; one is
  enough for JPEG's use pattern (synchronous, single outstanding).
- Extend shader to apply `JPEG_ORIENTATION` (90/180/270) as part of the
  blit transform — rotation collapses into the same UV math as the crop.
- libjpeg encodes straight from the mmap'd dma-buf into the gralloc
  BLOB. `mRgbaTemp` + `libyuv::ARGBRotate` deleted.

### 5. Delete packed-YUV paths

Target hw is Tegra K1 only (IMX179 / OV5693 — Bayer). `V4L2_PIX_FMT_UYVY`
/ `YUYV` branches in `BufferProcessor` and `ImageConverter::{UYVYToRGBA,
YUY2ToRGBA, UYVYToJPEG, YUY2ToJPEG}` are dead on our hardware — remove
them. If `ImageConverter` has no live methods left after this, fold the
remaining `RGBAToJPEG` call site directly into the JPEG path from (4)
and delete the class.

### 6. Purge the CPU fallback API surface

After (1)–(5) the following are unreachable — remove from the codebase:

- The entire `switch(srcBuf.stream->format)` block in
  `BufferProcessor::processOne` that runs post-`SW_WRITE_OFTEN`-lock.
- The `SW_WRITE_OFTEN` `GraphicBufferMapper::lock` itself on the hot path.
- `IspPipeline::process()` (synchronous CPU readback), and its override
  in `VulkanIspPipeline`.
- `IspPipeline::processSync()` — replaced by `processToGralloc` +
  `processToJpegBuffer` (new) as the only output paths.
- `mRgbaTemp` allocation (`Camera::ensureTempSize`) and the field.

### End state

`BufferProcessor::processOne` has exactly two output branches per buffer:

1. `RGBA_8888` → `processToGralloc(..., cropRect)` — GPU demosaic +
   optional crop/scale straight into gralloc.
2. `BLOB` → `processToJpegBuffer(..., cropRect, orientation)` — GPU
   demosaic + optional rotation into a mapped dma-buf, then libjpeg to
   the BLOB.

No `mRgbaTemp`, no `libyuv` on the hot path, no `ImageConverter::*YUV*`,
no `IspPipeline::process()`. Frame budget for zoomed / mismatched-
resolution preview stays within the ~45 ms zero-copy envelope.

### AF sweep note

AF still `SW_READ_OFTEN`-locks the gralloc output for Laplacian
measurement. That's a secondary read, not an ISP fallback, and it's
infrequent (sweep only). Left alone for this tier; revisit when the IPA
module lands (Tier 3) — ISP statistics will replace the CPU Laplacian
and that lock disappears too.

## Tier 2 — structural wins (M)

### `YUV_420_888` output (M)

Add `processToYuv420()` on `IspPipeline`. Vulkan / GLES already render
RGBA; adding NV12 is a pass of the existing shader or a libyuv
conversion. Advertise it in stream configs.

Unblocks CameraX `ImageAnalysis`, ML Kit, most third-party video
pipelines.

### Drain-to-latest in `V4l2Device::readLock` (S)

See [latency-and-buffers.md](latency-and-buffers.md) fix #1. Cuts
preview latency to 1 frame without touching queue depth or threading
model. ~50 lines.

### Graceful ISP backend fallback (S)

`Hw → Vulkan` on `init` failure. Currently we proceed with a broken
ISP. Cheap resilience. (CPU / GLES backends were removed — adding
either back purely as a fallback is not worth the maintenance cost.)

### JSON tuning file per sensor (M)

Load on HAL start, keyed by sensor model. Replaces all hardcoded
constants: VCM range, focal length, sensor area, default exposure / gain,
AF sweep parameters, sensor-specific ISP tuning. Makes adding a second
sensor a data-only change.

Borrowed directly from RPi — see
[open-source-references.md](open-source-references.md).

## Tier 3 — pipeline refactor (L)

### Request queue with dedicated capture thread (L)

Introduce `CaptureRequestQueue` and an async "capture worker" between
`processCaptureRequest` (producer) and V4L2 / ISP / gralloc (consumer).
Targets `PIPELINE_MAX_DEPTH ≈ 3`.

Gives us:

- Framework thread no longer blocks on DQBUF → ISP → JPEG serially.
- Sensor cadence decoupled from consumer pace.
- Foundation for proper AE precapture and reprocessing.

See [latency-and-buffers.md](latency-and-buffers.md) fix #2 and
[open-source-references.md](open-source-references.md) §AOSP V4L2 HAL.

### `DelayedControls` tracking (M)

Ring buffer mapping frame numbers to pending control values, with
per-control delay (2 for exposure / gain, 1 for VCM). Used to
populate result metadata with values that actually applied on the
captured frame, not the values most recently requested.

Prerequisite for truthful manual-exposure and HDR-bracketing support.

### 3A module with statistics feedback (L)

Pull the per-frame logic in `hal/3a/` behind an `IpaModule` with a
`process(StatsBuffer) → ControlUpdate` interface. Feed it statistics
from the ISP rather than the rendered preview.

- For `HwIspPipeline`: `mStatsHandle` is already allocated but
  unread — read it, parse ISP stats, feed IPA.
- For soft ISP: add a downscaled statistics pass to the Vulkan / GLES
  pipeline (a few hundred `uvec4` patches + histogram). Cheap on GPU.

Unblocks real AE, real AWB, face-aware AF, metering regions. Big payoff.

## Deferred / low priority

### `isp/hw/HwIspPipeline.cpp` (427 LOC)

Dead code per `memory/project_hw_isp_status.md` — kept as reference. Skip
until/unless we decide to revive the HW ISP path, at which point the
nvmap/nvhost ioctl structs (lines 20–70) extract to
`isp/hw/NvmapHelper.h` / `NvhostHelper.h`.

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

### Sandboxed 3A (optional)

Isolate 3A (and the `libnvisp_v3` blob) in a separate process with
Binder / Mojo IPC. A crash in the NVIDIA blob stops taking the HAL
down with it. Useful if the blob proves unstable; not worth the
complexity until it does.

## Suggested sequencing

1. **Tier 1.2** — short compliance PRs inside `CameraStaticMetadata`.
2. **Tier 1.5** — zero-copy everything. Shader work first (items 1–3),
   then JPEG (4), then the fallback-API purge (5–6).
3. **Tier 2** — drain-to-latest (quick latency win) and JSON tuning
   (cleanup that makes Tier 3 easier).
4. **Tier 3** — branch: request-queue refactor → DelayedControls →
   IPA module split.
5. **Tier 4** — discretionary.
