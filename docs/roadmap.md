# Roadmap

Prioritised list of improvements. Effort estimates are rough
(S = under a day, M = a few days, L = a week or more). Not a commitment
— a menu.

## Tier 1 — fix what's actively wrong (S–M)

### Echo request controls in result metadata (S)

Copy request keys verbatim into `processCaptureResult`:
`SENSOR_EXPOSURE_TIME`, `SENSOR_SENSITIVITY`, `SENSOR_FRAME_DURATION`,
`CONTROL_{AE,AWB,AF}_MODE`, `LENS_APERTURE`, `LENS_FOCAL_LENGTH`,
`CONTROL_CAPTURE_INTENT`, `JPEG_*`.

See [camera3-compliance.md](camera3-compliance.md) P0 for rationale.
Unlocks most CameraX clients.

### Report AE / AWB state (S)

Populate `CONTROL_AE_STATE` and `CONTROL_AWB_STATE` with at minimum
`INACTIVE` / `CONVERGED` / `LOCKED` transitions consistent with current
mode. No real algorithm required.

### Wire `notifyError` (S)

Helper `Camera::notifyError(frameNumber, stream, type)` called on every
early-return path in `processCaptureRequest`. Prevents frozen-preview
bugs.

### Populate `PIPELINE_MAX_DEPTH` and `PARTIAL_RESULT_COUNT` (S)

Honest values: `1` and `1` today. Bump after tier 3 lands.

### Fix `min_frame_duration` arrays (S)

`hal/Camera.cpp:229, 253` hardcode 60 fps. Replace with sensor-queried
max-fps-per-resolution so the framework stops requesting impossible
rates on modes that can only do 30.

## Tier 1.1 — refactor the monoliths (M)

The HAL violates its own design rules (one object per file; SRP;
DRY; no impl leaks) in several large files. Each one got so big that
every new Tier 1/2 task touches code that doesn't belong to what the
task is about. Before we keep piling features onto these files, we
split them.

This tier is both a concrete split plan and a standing rule: **every
file touched by later work gets audited against the rules as part of
that work**. New violations spotted after this tier opens go into the
same plan.

### Priority H

#### `hal/Camera.cpp` (1345 LOC)

Responsibilities bundled today:
1. Android Camera3 device callbacks (the C-facing ops table)
2. Static characteristics generation (~240 LOC in `staticCharacteristics()`)
3. Per-frame request pipeline (~500 LOC in `processCaptureRequest()`)
4. Autofocus state machine (`mAfSweep*`, `mAfSettleFrames`, sweep step/pick logic)
5. Temp RGBA buffer sizing (`mRgbaTemp`, `ensureTempSize`)
6. V4L2 control application (exposure/gain splitting, EV compensation)
7. JPEG buffer sizing + JPEG orientation rotation

Split into:
- `hal/Camera.{h,cpp}` — callback dispatcher + result routing only
- `hal/CameraStaticMetadata.{h,cpp}` — `buildStaticCharacteristics(Camera*, CameraMetadata&)`; all
  resolution/format/AE/AWB/AF mode enumeration. Also the home for the
  three `AVAILABLE_{REQUEST,RESULT,CHARACTERISTICS}_KEYS` arrays —
  having the static-chars build and the keys-arrays in the same file
  makes the source-of-truth rule enforceable by reading one file.
  This subsumes Tier 1 item "Build AVAILABLE_*_KEYS" (moved here so
  we don't pile another 40+ lines of static arrays into the existing
  mess). Sensor calibration keys (`BLACK_LEVEL_PATTERN`, `WHITE_LEVEL`,
  `COLOR_FILTER_ARRANGEMENT`, colour/forward matrices,
  `REFERENCE_ILLUMINANT{1,2}`, `NOISE_PROFILE`) also land here as part
  of this extraction — per-sensor values pulled from `SensorConfig`
  (or the JSON tuning file once it lands in Tier 2), AVAILABLE_KEYS
  array extended in the same commit.
- `hal/AutoFocusController.{h,cpp}` — sweep state machine; interface
  `onTriggerStart(mode)`, `onFrame(rgba) → ControlUpdate`, `cancel()`
- `hal/ExposureControl.{h,cpp}` — request→V4L2 exposure/gain split, EV
  compensation, applied-value report
- `hal/JpegEncoder.{h,cpp}` — own the rotation+encode+BLOB wrap (today
  scattered in `processCaptureRequest` and `ImageConverter::RGBAToJPEG`)
- Temp RGBA buffer disappears entirely once Tier 1.5 lands

#### `isp/vulkan/VulkanIspPipeline.cpp` (1399 LOC)

Bundled today:
1. Vulkan instance/device/queue/descriptor-set bootstrap
2. Compute + graphics pipeline creation (demosaic shader, blit shader)
3. Per-frame command-buffer recording + submit
4. Gralloc image caching (`GrallocEntry` struct, per-`native_handle_t*` lookup)
5. ISP params struct (`IspParams`: CCM, WB, gamma LUT indices)
6. Gamma LUT math (duplicated in `ImageConverter`)

Split into:
- `isp/vulkan/VulkanIspPipeline.{h,cpp}` — orchestration of the per-frame
  path only (`process*`, `prewarm`, `waitForPreviousFrame`)
- `isp/vulkan/runtime/VulkanDeviceState.{h,cpp}` — instance/device/queue/
  descriptor-set lifecycle; RAII-owned by `VulkanIspPipeline`
- `isp/vulkan/VulkanGrallocCache.{h,cpp}` — `GrallocEntry` + per-handle
  cache; takes a `VulkanDeviceState&` on construction
- `isp/IspParams.{h,cpp}` — `IspParams` struct, default/template helpers;
  reusable by the future IPA module
- `isp/sensor/IspCalibration.{h,cpp}` — per-sensor CCM tables behind static
  accessors. (Gamma LUT turned out to be dead on both backends: the
  Vulkan shader computes sRGB inline via `pow()`, the CPU path is gone;
  no LUT is needed anywhere.)

#### `image/ImageConverter.cpp`

What it had before Tier 1.1 started: YUY2/UYVY → RGBA/JPEG, CPU Bayer
demosaic with its own WB/CCM/gamma, a duplicate set of CCM tables, a
duplicate gamma LUT init, and global AWB state `sPrevWbR/G/B`. The
CPU Bayer path had zero live callers — Vulkan handled every Bayer
request through `processSync` / `processToGralloc`.

The ISP-related portions were removed wholesale as part of the
`IspCalibration` extraction step: `BayerToRGBA` (+ `demosaicIspLine`,
`bayerPattern`, `bayerIs16bit`), the duplicate CCM tables, the
duplicate gamma LUT, and the global AWB state all gone. The
Bayer-only fields in `ConvertTask::Data` went with them.

What remains: `UYVYToRGBA`, `YUY2ToRGBA`, `UYVYToJPEG`, `YUY2ToJPEG`,
`RGBAToJPEG` — pure format conversion, no ISP math. Dies in Tier 1.5
(Tegra K1 is Bayer-only and JPEG encode moves to a mapped dma-buf).

### Priority M

#### `v4l2/V4l2Device.cpp` (808 LOC)

- Extract `Resolution` struct → `v4l2/Resolution.h` (or `util/Resolution.h`
  if it ends up shared with static-metadata generation).
- Consider extracting focuser-subdev control → `v4l2/V4l2Focuser.{h,cpp}`
  if AF moves out into its own module (it will — see `AutoFocusController`
  above).

#### `util/DbgUtils.h` (267 LOC)

Three unrelated template classes (`AutoLogCall`, `FpsCounter`, `Benchmark`)
in one header. Split into `util/AutoLogCall.h`, `util/FpsCounter.h`,
`util/Benchmark.h`. Macro wrappers can stay in a thin `util/DbgUtils.h`
that includes the three.

### Priority L

#### `isp/hw/HwIspPipeline.cpp` (427 LOC)

Dead code per `memory/project_hw_isp_status.md` — kept as reference. Skip
until/unless we decide to revive the HW ISP path, at which point the
nvmap/nvhost ioctl structs (lines 20–70) extract to
`isp/hw/NvmapHelper.h` / `NvhostHelper.h`.

### Ordering

1. `isp/IspLut` extraction first — kills the gamma/CCM duplication
   between Vulkan and CPU paths; very mechanical.
2. `hal/AutoFocusController` and `hal/CameraStaticMetadata` — both
   blocked by nothing; each cuts ~250–350 LOC out of `Camera.cpp`.
3. `VulkanIspPipeline` split — after IspLut lands (so the split doesn't
   pick up a dead copy of the gamma code).
4. `hal/ExposureControl`, `hal/JpegEncoder` — pull out after AF split to
   get `Camera.cpp` under ~400 LOC.
5. `v4l2/Resolution.h` + `DbgUtils.h` split — low-priority cleanup pass.
6. `ImageConverter` final collapse — happens as part of Tier 1.5 when
   all CPU paths die.

Each extraction is its own PR. No behaviour changes: pure moves + rename.

## Tier 1.5 — eliminate CPU fallbacks, zero-copy everything (M)

Today zero-copy only triggers when **all** of these hold simultaneously:
RGBA_8888 output, stream size equals V4L2 capture size, no zoom, Bayer
input, first RGBA buffer of the request. Anything else falls back to a
CPU path that demosaics → reads back → runs libyuv → encodes — killing
the ~45 ms frame budget and re-introducing the 25 ms blocklinear detile
cost on the `SW_WRITE_OFTEN` gralloc lock.

Goal of this tier: **delete every CPU fallback** in
`processCaptureRequest`. If a case cannot be serviced on the GPU, we
don't support it.

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
/ `YUYV` branches in `Camera.cpp` and `ImageConverter::{UYVYToRGBA,
YUY2ToRGBA, UYVYToJPEG, YUY2ToJPEG}` are dead on our hardware — remove
them. If `ImageConverter` has no live methods left after this, fold the
remaining `RGBAToJPEG` call site directly into the JPEG path from (4)
and delete the class.

### 6. Purge the CPU fallback API surface

After (1)–(5) the following are unreachable — remove from the codebase:

- The entire `switch(srcBuf.stream->format)` block in
  `processCaptureRequest` that runs post-`SW_WRITE_OFTEN`-lock.
- The `SW_WRITE_OFTEN` `GraphicBufferMapper::lock` itself on the hot path.
- `IspPipeline::process()` (synchronous CPU readback), and its override
  in `VulkanIspPipeline`.
- `IspPipeline::processSync()` — replaced by `processToGralloc` +
  `processToJpegBuffer` (new) as the only output paths.
- `mRgbaTemp` allocation (`Camera::ensureTempSize`) and the field.

### End state

`processCaptureRequest` has exactly two output branches per buffer:

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

Pull AF/AE/AWB out of `hal/Camera.cpp` into a separate `IpaModule` with a
`process(StatsBuffer) → ControlUpdate` interface. Feed it statistics
from the ISP rather than the rendered preview.

- For `HwIspPipeline`: `mStatsHandle` is already allocated but
  unread — read it, parse ISP stats, feed IPA.
- For soft ISP: add a downscaled statistics pass to the Vulkan / GLES
  pipeline (a few hundred `uvec4` patches + histogram). Cheap on GPU.

Unblocks real AE, real AWB, face-aware AF, meteing regions. Big payoff.

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

1. Tier 1 in parallel with Tier 1.1 — each Tier 1 item that lands in
   `Camera.cpp` should leave the code in a cleaner shape than it
   found. New files created during Tier 1.1 absorb the logic; `Camera.cpp`
   shrinks on every merge.
2. Tier 1.1 extractions driven opportunistically by Tier 1 work;
   standalone PRs for the ones not motivated by a feature (IspLut,
   DbgUtils split, Resolution header).
3. Tier 1.5 (zero-copy everything) after Tier 1 + the
   `VulkanIspPipeline` split — the split makes the shader changes
   small instead of massive.
4. Before Tier 3: land **drain-to-latest** (Tier 2) for an immediate
   latency win, and **JSON tuning** for a cleanup that makes Tier 3
   easier.
5. Tier 3 in a branch, with the request-queue refactor first (no
   behaviour change, just decoupling), then DelayedControls, then the
   IPA module split.
6. Tier 4 is discretionary.
