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

### Build `AVAILABLE_*_KEYS` arrays (S)

Mechanical. Enumerate every key the HAL reads from requests, writes
into results, or exposes in characteristics, and publish the three
arrays. Improves framework cooperation.

### Populate `PIPELINE_MAX_DEPTH` and `PARTIAL_RESULT_COUNT` (S)

Honest values: `1` and `1` today. Bump after tier 3 lands.

### Fix `min_frame_duration` arrays (S)

`Camera.cpp:229, 253` hardcode 60 fps. Replace with sensor-queried
max-fps-per-resolution so the framework stops requesting impossible
rates on modes that can only do 30.

### Sensor calibration keys (S–M)

Ship `BLACK_LEVEL_PATTERN`, `WHITE_LEVEL`,
`COLOR_FILTER_ARRANGEMENT`, the colour / forward matrices, and
`NOISE_PROFILE`. Values come from sensor datasheet or vendor tuning.
Cheap to add, unblocks DNG capture.

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

`Hw → Vulkan → GLES → CPU` on `init` failure. Currently we proceed with
a broken ISP. Cheap resilience.

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

Pull AF/AE/AWB out of `Camera.cpp` into a separate `IpaModule` with a
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

1. Ship Tier 1 in one or two PRs. Low risk, unlocks apps today.
2. Before diving into Tier 3: land **drain-to-latest** (Tier 2) for an
   immediate latency win, and **JSON tuning** for a cleanup that makes
   Tier 3 easier.
3. Tier 3 in a branch, with the request-queue refactor first (no
   behaviour change, just decoupling), then DelayedControls, then the
   IPA module split.
4. Tier 4 is discretionary.
