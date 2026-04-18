# Camera3 compliance gaps

This HAL declares `ANDROID_INFO_SUPPORTED_HARDWARE_LEVEL_LIMITED`
(`hal/Camera.cpp:378`) but is missing several metadata keys and behaviours
that `LIMITED` implementations are expected to provide. This document
enumerates the gaps, ordered by how badly they break real applications.

## Severity legend

- **P0 — Breaks basic apps.** Framework or CameraX/Camera2 client will
  visibly fail, hang, or render garbage.
- **P1 — Breaks specific features.** The app works in the golden path
  but specific features (DNG, ML analysis, precapture metering) don't.
- **P2 — CTS / quality.** Needed for CTS compliance or subjectively
  better image quality, but apps function without it.

## P0 — Must fix for broadly usable HAL

### AE/AWB state never reported in result metadata

Nothing writes `ANDROID_CONTROL_AE_STATE`, `ANDROID_CONTROL_AWB_STATE`,
or `ANDROID_CONTROL_AF_STATE` transitions beyond the AF sweep case.
CameraX's ImageCapture flow waits on `AE_STATE == CONVERGED` before
taking a still — with no state reported, it will hang or time out.

**Fix:** even a trivially correct `AE_STATE = INACTIVE` / `AWB_STATE =
INACTIVE` (when modes are OFF) is better than nothing. For `AWB_MODE_AUTO`
report `LOCKED` if the framework set `AWB_LOCK`, else `CONVERGED`.

### Requested controls not echoed in result metadata

The result built around `hal/Camera.cpp:1132+` is missing:

- `ANDROID_SENSOR_EXPOSURE_TIME`
- `ANDROID_SENSOR_SENSITIVITY`
- `ANDROID_SENSOR_FRAME_DURATION`
- `ANDROID_CONTROL_AE_MODE`
- `ANDROID_CONTROL_AWB_MODE`
- `ANDROID_CONTROL_AF_MODE`
- `ANDROID_CONTROL_CAPTURE_INTENT`
- `ANDROID_LENS_APERTURE` / `ANDROID_LENS_FOCAL_LENGTH`
- `ANDROID_JPEG_ORIENTATION` / `ANDROID_JPEG_QUALITY`

**Fix:** echo what the request asked for. Apps compare request vs result
to know what actually applied — if a key is absent, Camera2 throws.

### `ANDROID_REQUEST_AVAILABLE_*_KEYS` missing

- `ANDROID_REQUEST_AVAILABLE_REQUEST_KEYS`
- `ANDROID_REQUEST_AVAILABLE_RESULT_KEYS`
- `ANDROID_REQUEST_AVAILABLE_CHARACTERISTICS_KEYS`

The framework validates requests and results against these lists.
Absence means "we support nothing", which the framework may interpret as
"no settings are honoured" and reject the device for some features.

**Fix:** build the three arrays with every key the HAL actually
reads / writes. This is tedious but mechanical.

### `YUV_420_888` output not supported

Only `RGBA_8888` and `BLOB` are offered
(`ANDROID_SCALER_AVAILABLE_STREAM_CONFIGURATIONS` in `hal/Camera.cpp` around
line 220). `YUV_420_888` is mandatory for `LIMITED` and is the format
CameraX's `ImageAnalysis`, ML Kit, and most third-party video pipelines
consume. Without it, CameraX falls back to unusable paths or refuses to
open the camera.

**Fix:** add a `processToYuv420()` path on `IspPipeline` (Vulkan/GLES
already output in RGBA; converting to NV12/YV12 is a short extra shader
pass or a libyuv call). Expose the format in the stream config array.

### `notifyError` path not wired up

`grep notifyError` returns zero uses. Any V4L2 read failure
(`readLock()` returning `NULL`, `hal/Camera.cpp:866`) returns
`NOT_ENOUGH_DATA` silently. Framework expects `CAMERA3_MSG_ERROR_REQUEST`
or `CAMERA3_MSG_ERROR_BUFFER` on such failures, otherwise it waits
indefinitely for the result.

**Fix:** wrap a `notifyError(frame_number, stream, type)` helper and call
it on every early-return path in `processCaptureRequest`.

## P1 — Breaks specific features

### Sensor calibration metadata absent

Missing keys prevent DngCreator from producing valid DNGs and prevent
colour-critical apps from doing their own correction:

- `ANDROID_SENSOR_INFO_COLOR_FILTER_ARRANGEMENT`
- `ANDROID_SENSOR_BLACK_LEVEL_PATTERN`
- `ANDROID_SENSOR_INFO_WHITE_LEVEL`
- `ANDROID_SENSOR_CALIBRATION_TRANSFORM{1,2}`
- `ANDROID_SENSOR_COLOR_TRANSFORM{1,2}`
- `ANDROID_SENSOR_FORWARD_MATRIX{1,2}`
- `ANDROID_SENSOR_REFERENCE_ILLUMINANT{1,2}`
- `ANDROID_SENSOR_NOISE_PROFILE`

**Fix:** the calibration matrices are per-sensor data. Collect them from
the sensor datasheet (or from `libnvisp_v3`'s tuning binary if it
exposes them) and store per-sensor in a tuning struct. `BLACK_LEVEL` and
`CFA` are cheap and purely sensor-dependent.

### `ANDROID_REQUEST_PIPELINE_MAX_DEPTH` / `PARTIAL_RESULT_COUNT` absent

Without `PIPELINE_MAX_DEPTH` the framework assumes a depth of 1 and will
not pre-queue requests. This interacts badly with the frame-latency
problem in [latency-and-buffers.md](latency-and-buffers.md).

**Fix:** report the real in-flight request depth your HAL guarantees.
For the current single-threaded implementation that's `1` (honest). After
introducing a request queue (see [roadmap.md](roadmap.md)) bump to `3`–`4`.

### `AE_PRECAPTURE_TRIGGER` ignored

`ANDROID_CONTROL_AE_PRECAPTURE_TRIGGER` is never read. CameraX's
ImageCapture sets this on still capture to force AE to converge before
the shot. We silently ignore it, so still capture happens mid-metering
and comes out misexposed.

**Fix:** wire it into an AE state machine (currently there is no real AE
loop, so this is really "implement AE first").

### Metering regions unsupported

`ANDROID_CONTROL_MAX_REGIONS = {0, 0, 0}` (line ~318). All tap-to-focus /
tap-to-expose UIs in third-party apps become no-ops.

**Fix:** after introducing a real AE / AF region handler, bump the AF
region count to `1`. AE / AWB regions can remain 0 until the
corresponding algorithms exist.

### Face detection disabled

`STATISTICS_INFO_MAX_FACE_COUNT = 0`, `STATISTICS_FACE_DETECT_MODE_OFF`.
This is fine for a bare-bones HAL; flagging it so the roadmap is honest.

### ZSL and reprocessing rejected

Input streams are accepted and ignored (`hal/Camera.cpp:762-765`); ZSL-flagged
streams are rejected outright (`hal/Camera.cpp:570-571`). Both are optional
for `LIMITED` but useful for still-capture latency.

### HFR / constrained high-speed missing

No `ANDROID_CONTROL_AVAILABLE_HIGH_SPEED_VIDEO_CONFIGURATIONS`.

## P2 — CTS and polish

### Sensor characteristics are partially faked

`hal/Camera.cpp:160-180` uses hardcoded constants: 5×5 mm physical sensor
area (scaled by aspect ratio), 3.3 mm focal length, fixed minimum focus
distance. Good enough for preview; wrong for apps using physical
parameters (depth estimation, augmented reality, focal plane metadata
for post-processing).

**Fix:** pull real values from a per-sensor tuning file. See
[open-source-references.md](open-source-references.md) on how RPi / Intel
structure these.

### Tonemap keys absent

`ANDROID_TONEMAP_*` — tonemap mode, available tonemap modes, curve
points. Apps that want linear response (HDR capture pipelines) cannot
disable the built-in tonemap.

### Hot pixel / edge / noise reduction keys absent

`ANDROID_HOT_PIXEL_*`, `ANDROID_EDGE_*`, `ANDROID_NOISE_REDUCTION_*`.
Most apps don't touch these but CTS checks the keys exist with at least
one supported mode.

### Stream-configuration min frame durations assume 60 fps

`hal/Camera.cpp:229, 253` — hardcoded `1_000_000_000 / 60`. On sensors that
can't actually do 60 fps at full resolution, framework will request
impossible frame rates.

**Fix:** query the sensor for supported `frame_length` → derive real
`min_frame_duration` per resolution.

## Summary table

| Category            | Coverage |
|---------------------|----------|
| Static metadata     | ~50% of LIMITED expected keys |
| Request acceptance  | ~70% (templates & basic controls OK, precapture missing) |
| Result metadata     | ~30% (timestamp, AF state; nothing else) |
| Streams             | ~60% (RGBA + BLOB; no YUV_420_888, no ZSL) |
| 3A algorithms       | ~20% (AF sweep works, AE/AWB stubbed) |
| Error callbacks     | 0% (`notifyError` unused) |
| Threading model     | Single-threaded, pipeline depth = 1 |

Honest hardware-level claim given the above: between `EXTERNAL` and
`LIMITED`. Reaching real `LIMITED` takes most of the P0 + P1 list.
`FULL` / `LEVEL_3` require manual-mode guarantees (precise exposure /
sensitivity / frame duration latency) that V4L2 + Tegra VI cannot
trivially provide without `DelayedControls`-style tracking — see
[latency-and-buffers.md](latency-and-buffers.md).
