# Camera3 / Camera2 contract surface

The HAL claims `CAMERA_DEVICE_API_VERSION_3_4` /
`CAMERA_MODULE_API_VERSION_2_3` /
`ANDROID_INFO_SUPPORTED_HARDWARE_LEVEL_LIMITED` /
`ANDROID_REQUEST_AVAILABLE_CAPABILITIES = [BACKWARD_COMPATIBLE]`.
The full Camera2 contract surface that flows from those declarations
is documented in the `project_hal34_compliance` developer note;
this doc covers what's **not** claimed and what each missing
capability would unlock.

## Capabilities not claimed and what each would mandate

### `RAW`

Would require `BLACK_LEVEL_PATTERN`, `WHITE_LEVEL`,
`COLOR_TRANSFORM_{1,2}`, `FORWARD_MATRIX_{1,2}`,
`CALIBRATION_TRANSFORM_{1,2}`, `REFERENCE_ILLUMINANT{1,2}`,
`NOISE_PROFILE`, plus a RAW16 (or RAW_OPAQUE) stream producer
that bypasses demosaic.

**Why we don't claim it:** Mi Pad 1 use case has no DNG workflow.
The calibration data (forward / calibration matrices, noise
profile) isn't in the stock NVIDIA `.isp` files — would require a
manual calibration session on the target hardware.

### `MANUAL_SENSOR`

Would require precapture-trigger handling, per-frame sensor
settings round-trip with strict timing guarantees,
`SENSOR_INFO_EXPOSURE_TIME_RANGE` / `SENSITIVITY_RANGE` honoured
without remap, and `MAX_NUM_OUTPUT_RAW > 0` (entangled with `RAW`
capability).

**Why we don't claim it:** the strict precapture-trigger semantics
+ no-remap timing guarantees are more than `BasicIpa` provides.
Manual exposure / gain via `SENSOR_EXPOSURE_TIME` /
`SENSOR_SENSITIVITY` in the AE_MODE=OFF path is honoured on a
best-effort basis through `ExposureControl`, but the strict
`MANUAL_SENSOR` contract isn't promised.

### `MANUAL_POST_PROCESSING`

Would require `COLOR_CORRECTION_TRANSFORM` / `GAINS` honoured
per-frame (we honour `GAINS` in AWB_OFF mode but ignore
`TRANSFORM`), `TONEMAP_CURVE_*` (we report `TONEMAP_MODE = FAST`
only), `EDGE_MODE` / `NOISE_REDUCTION_MODE` actually doing
something (we advertise OFF-only).

**Why we don't claim it:** post-processing knobs the HAL doesn't
implement. Tone curves, edge enhancement, NR all live in
`reserved.*` of the per-module tuning JSON; no shader stage
consumes them yet.

### `BURST_CAPTURE`

Would require declared min frame durations met across rapid
sequences of stills, `SYNC_MAX_LATENCY = PER_FRAME_CONTROL` (we
report `UNKNOWN`).

**Why we don't claim it:** no targeted timing guarantees on the
JPEG path. JpegWorker is async but libjpeg encode is ~100-150 ms
per 8 MP shot on the Cortex-A15, far above frame_period — bursting
through the BLOB FIFO gate would back up.

### `READ_SENSOR_SETTINGS`

Would require advertising the actually-applied
`SENSOR_EXPOSURE_TIME` / `SENSITIVITY` / `FRAME_DURATION` /
`ROLLING_SHUTTER_SKEW` / `TIMESTAMP_SOURCE = REALTIME` per frame.

**Why we don't claim it:** `TIMESTAMP_SOURCE` is `UNKNOWN`,
rolling-shutter skew not measured. Exposure / sensitivity round-
tripping through `DelayedControls` is honest but the rest of the
contract isn't.

### `PRIVATE_REPROCESSING` / `YUV_REPROCESSING`

Would require `MAX_NUM_INPUT_STREAMS >= 1`,
`SCALER_AVAILABLE_INPUT_OUTPUT_FORMATS_MAP` populated,
`Request::inputBuffer` honoured by routing through ISP.

**Why we don't claim it:** the inputBuffer slot is reserved in
`PipelineContext` but unwired. ZSL ring + reprocess pipeline is a
deliberate future feature, not blocked on infrastructure.

### `CONSTRAINED_HIGH_SPEED_VIDEO`

Would require a high-fps sensor mode (≥ 120 fps, no manual
controls) and `operation_mode = CONSTRAINED_HIGH_SPEED_MODE`
handling on `configureStreams`.

**Why we don't claim it:** the sensors don't expose a high-fps
mode the V4L2 driver has surfaced; the `StreamConfig::normalize`
path explicitly rejects the non-NORMAL operation mode.

### `DEPTH_OUTPUT`

Would require `DEPTH16` / `DEPTH_POINT_CLOUD` / `Y16` stream
configurations + per-pixel depth backing (real depth, not a
synthetic stand-in — apps assume the values are calibrated mm).

**Why we don't claim it:** no depth hardware. Mocha is monocular
RGB; no stereo pair, no time-of-flight sensor, no IR projector
for structured light. Software depth-from-monocular (NN-based or
focus-stack-based) wouldn't satisfy the contract's accuracy
expectation, and lying about the format would mislead apps.

## Closed gaps (kept as a historical reference)

The following items used to be open and were closed during the
HAL3.4 / Camera2 compliance pass (commit `f34d870` and around):

- **AE / AWB / AF state in result metadata** — `BasicIpa` reports
  honest `INACTIVE / SEARCHING / CONVERGED / LOCKED` on AE,
  `INACTIVE / CONVERGED / LOCKED` on AWB; AF lifecycle is owned
  by `AutoFocusController` and reported per-frame.
- **Requested controls echoed in result metadata** — every key
  in `ANDROID_REQUEST_AVAILABLE_REQUEST_KEYS` round-trips through
  `ResultMetadataBuilder`.
- **`ANDROID_REQUEST_AVAILABLE_*_KEYS` arrays** — populated for
  request / result / characteristics; CameraX feature-availability
  probes pass.
- **`YUV_420_888` output** — produced by `VulkanYuvEncoder` (RGBA
  → NV12 compute), repacked into the gralloc layout `lockYCbCr`
  asks for via libyuv (NV12 / I420 / YV12; NV21 returns `NO_INIT`).
- **Per-mode `min_frame_duration`** — derived from
  `VIDIOC_ENUM_FRAMEINTERVALS`, fallback to 30 fps when the driver
  doesn't advertise.
- **`partial_result = 1`** on every result; `PARTIAL_RESULT_COUNT
  = 1` advertised.
- **HAL3.2 ABI** — `register_stream_buffers` and
  `get_metadata_vendor_tag_ops` set to NULL (deprecated form);
  consumer gralloc usage flags preserved.
- **HAL3.3 ABI** — `camera3_stream_configuration_t::operation_mode
  == NORMAL_MODE`, `camera3_stream_t::rotation == ROTATION_0`,
  `data_space != DEPTH` validated and rejected with `BAD_VALUE`
  when out of contract.
- **AE_LOCK + EV-comp** — EV compensation honoured on both auto
  and locked paths, EMA-smoothed across the lock so big EV steps
  don't mid-rolling-shutter-tear.
- **AF_REGIONS** — parsed into a patch-grid `FocusRoi` (5×5
  minimum around tap centre); the same ROI feeds AF metric
  computation, AE spot-meter region, and the NEON encoder's Sobel
  / greenSq window.
- **Sensor calibration** — physical size, focal length, min focus
  distance, sensor orientation, bayer pattern from per-module
  tuning JSON. VCM range from the focuser subdev's
  `VIDIOC_QUERYCTRL`.
- **`SENSOR_INFO_EXPOSURE_TIME_RANGE` / `SENSITIVITY_RANGE`** —
  queried from V4L2 driver, no compile-time hardcodes.

The full enumerator of advertised keys + capabilities lives in
`hal/metadata/CameraStaticMetadata.cpp`; the request keys that
round-trip into result metadata are in
`hal/metadata/RequestTemplateBuilder.cpp` and
`hal/metadata/ResultMetadataBuilder.cpp`.
