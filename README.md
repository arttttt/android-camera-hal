# Tegra K1 / Mi Pad 1 Camera HAL

V4L2-based Android Camera HAL for **Xiaomi Mi Pad 1** (codename
`mocha`, Tegra K1, single-SMX Kepler GK20A) running LineageOS 14.1 /
Android 7.1.2. Software ISP runs on the GPU via Vulkan compute +
fragment, 3A statistics on ARMv7 NEON. No closed NVIDIA blob (no
`libnvodm_imager` / `libnvcamerautils`); all processing is ours from
the raw Bayer pixel onward.

## Status

| Property | Value |
|----------|-------|
| HAL device API | `CAMERA_DEVICE_API_VERSION_3_4` |
| Module API | `CAMERA_MODULE_API_VERSION_2_3` |
| Camera2 hardware level | `LIMITED` |
| Capabilities | `BACKWARD_COMPATIBLE` |
| Cameras | IMX179 rear (8 MP RGGB), OV5693 front (5 MP BGGR) |
| Output formats | RGBA_8888, YCbCr_420_888 (NV12), BLOB (JPEG) |
| Output cap | 16:9 ≤ 1920×1080 (per stream) |
| Live throughput | ~30 fps preview at 1080p with full 3A active |

The HAL implements the Camera2 contract well enough that Open Camera,
the stock LineageOS camera and Libre Camera all run under Camera2 API
(not the legacy Camera1 emulation path). Photo, video record, and
preview are all working.

## Features

### Pipeline

- **Zero-copy V4L2 → Vulkan capture.** V4L2 in `V4L2_MEMORY_DMABUF`
  mode writes directly into Vulkan-allocated input ring buffers
  (4-slot, exported via `vkGetMemoryFdKHR` / `OPAQUE_FD`). No CPU
  memcpy on the input side.
- **GPU demosaic + blit.** Compute shader (Malvar-He-Cutler) with a
  20×20 cooperative shared-memory Bayer tile demosaics into a per-slot
  RGBA scratch image. Fragment shader samples the scratch via a
  `sampler2D` and writes the gralloc framebuffer through fragment ROP
  (the only Tegra K1 path that produces tiler-correct output to a
  `VK_ANDROID_native_buffer` image — compute-store goes through a
  swizzle and produces garbage).
- **Multi-stream produce-once.** One demosaic per frame; per-output
  blits / encodes / copies share a single `vkQueueSubmit`. Per-output
  release fences come from `vkQueueSignalReleaseImageANDROID`.
- **Asynchronous request pipeline.** Six worker threads per camera:
  - `RequestThread` — drains the framework's `processCaptureRequest`
    queue, deep-copies settings into a `PipelineContext`. Returns
    in < 1 ms.
  - `CaptureThread` (V4L2) — DQBUF off the V4L2 queue, drain-to-latest
    so preview never reaches into a stale frame.
  - `PipelineThread` — owns the GPU submit ring; fence-fd polled for
    completion (no `vkWaitForFences` on the hot path).
  - `ResultThread` — `process_capture_result` dispatch + Bayer flush
    + in-flight tracker remove; gates BLOB-bearing requests so
    Camera3's monotonic frame_number ordering survives async JPEG.
  - `JpegWorker` — libjpeg encode off the hot path, snapshot ring of
    host-mapped Vulkan buffers feeds it.
  - `StatsWorker` — NEON 3A statistics over the raw Bayer slot,
    progressive across `phaseCount` submits to keep peak CPU low.
- **Framework `acquire_fence` honoured** as a binary `VkSemaphore`
  via `VK_KHR_external_semaphore_fd` — the GPU submit waits on the
  framework's producer fence instead of blocking the recording
  thread on it.

### 3A

- **AE** — open-loop target in absolute EV space (µs at unity gain),
  single-pole LPF toward target each frame. Two parallel candidate
  ratios: `ratioMean = setpoint / luma` chases NVIDIA's
  `MeanAlg.{Higher,Lower}Target` midpoint (gamma-decoded into the
  linear-luma domain the metric lives in); `ratioHighlight =
  highlightCap / IQM_top2%` keeps the brightest 2% of post-WB max-
  of-channels patches under cap. Strictest wins (RPi / libcamera
  convention). Both clamped to the tuning's MaxFstopDelta envelope
  as a per-frame rate limit. Spot metering inside the active focus
  ROI (default centre 8×8 patches, tap-to-focus moves it). 2 %
  absolute-deviation dead-band freezes the LPF at convergence.
  Optional asymmetric LPF speed (faster pole when `|target/filtered
  − 1| < closeSpeedZone`) configurable per sensor via
  `active.hal_overrides.ae.close_speed_zone`. EV compensation as a
  multiplicative offset on the target. AE_LOCK holds converged
  exposure / gain, EV steps under lock are EMA-smoothed so a hard
  EV jump doesn't tear mid-rolling-shutter. Exposure / gain split
  via `SensorConfig::splitExposureGain` (prefers exposure up to
  default frame_length, then gain).
- **AWB** — gray-world over a 16×16 patch grid (rgbMean from NEON
  stats), saturated / near-black patches filtered. **96-valid-patch
  confidence gate**: below it, lastWb EMA-relaxes back to the
  per-CCT-calibrated daylight prior — gate is "trust the calibrated
  neutral over a stale gray-world reading on a noise / non-neutral
  subset". CCT estimated via NVIDIA's `awb.v4` U → CCT polynomial,
  CCM LERP'd across calibrated brackets.
- **AF** — CDAF state machine (`Idle → Coarse1 → [Coarse2] → Fine
  → Settle`). Score is `Σ(Gx² + Gy²) / Σ I²` (exposure-invariant
  Tenengrad ratio) computed in NEON over the focus ROI. Continuous
  AF retriggers on a multi-channel scene snapshot (focusMetric +
  RGB-mean over centre 8×8). Tap-to-focus parses
  `ANDROID_CONTROL_AF_REGIONS`, expands to a 5×5-patch minimum.
  AF holds AE+AWB lock across a sweep so the score curve isn't
  distorted by chasing brightness mid-scan.

### Camera2 contract

- `AVAILABLE_REQUEST_KEYS` / `AVAILABLE_RESULT_KEYS` /
  `AVAILABLE_CHARACTERISTICS_KEYS` populated.
- `REQUEST_AVAILABLE_CAPABILITIES = [BACKWARD_COMPATIBLE]`.
- `MAX_NUM_OUTPUT_STREAMS = {RAW=0, PROCESSED=2, STALLING=1}`
  (API 25 single-array form + API 26+ split scalars under one
  `#ifdef`).
- `SYNC_MAX_LATENCY = UNKNOWN`.
- Per-stage `AVAILABLE_*_MODES` for EDGE / HOT_PIXEL /
  NOISE_REDUCTION / SHADING / TONEMAP / COLOR_CORRECTION_ABERRATION /
  LENS_INFO_AVAILABLE_OPTICAL_STABILIZATION /
  SENSOR_AVAILABLE_TEST_PATTERN / STATISTICS_INFO_AVAILABLE_*.
- HAL3.3 contract validates `camera3_stream_configuration_t::
  operation_mode == NORMAL_MODE`, `camera3_stream_t::rotation == 0`,
  rejects `data_space == DEPTH`.
- HAL3.2 contract: `register_stream_buffers` and
  `get_metadata_vendor_tag_ops` are NULL'd; `partial_result = 1`
  on every result.
- AE / AWB / AF state reported per-frame in result metadata.

### Tuning

- Per-module JSON at `/vendor/etc/camera/tuning/<sensor>_<integrator>.json`.
- Source: stock NVIDIA `.isp` files (Mi Pad 1 vendor blob) converted
  by `tools/isp_to_json.py`, split into an `active` section (paths
  the HAL consumes) and a `reserved` section (preserved verbatim for
  future stages). Hardware facts (physical size, focal length, min
  focus distance, bayer pattern, sensor orientation) live in
  `tuning/_module_<sensor>_<integrator>.json` and are merged at
  conversion time.
- HAL-specific knobs that don't exist in NVIDIA `.isp` (currently the
  AE close-speed-zone width per sensor) live under an
  `active.hal_overrides` sub-section in the same per-module JSON,
  carved out so `tools/isp_to_json.py` preserves them verbatim across
  regenerations from the upstream `.isp`.
- Live consumers: `AutoFocusController`, `SensorTuning::ccmForCctQ10`,
  `DemosaicCompute` shader (optical-black bias), `BasicIpa` AE+AWB,
  `CameraStaticMetadata` (per-module geometry).
- Reserved (not yet consumed): lens shading, full AE VFRTable, full
  AWB CCT LUT, noise reduction sets, tone curves, sharpness filters.

### Output formats and stream sizing

- Advertised resolution set is filtered to **16:9 with a 1920×1080
  ceiling** — the ceiling tracks what the SW ISP can sustain at
  frame rate on this GPU. Below the ceiling every V4L2-supported
  16:9 mode is exposed.
- Auxiliary streams the framework configures at non-advertised sizes
  (e.g. MediaRecorder 320×240 thumbnail callback, face-detect
  inputs) are accepted up to the same dimension cap regardless of
  aspect.
- `IMPLEMENTATION_DEFINED` resolves by usage flags:
  `HW_VIDEO_ENCODER` → `YCbCr_420_888`, everything else →
  `RGBA_8888`. NV12 / I420 / YV12 chroma layouts are repacked from
  the NV12 the GPU produces; NV21 returns `NO_INIT`.

## Architecture (high level)

```
                        Android camera framework
                                  │ Camera3 ops
                          ┌───────▼───────┐
                          │     Camera    │
                          └───┬───────┬───┘
                              │       │
            ┌─────────────────┘       └─────────────┐
            ▼                                       ▼
    RequestThread                          per-camera helpers
    ↳ PipelineContext queue                AutoFocusController
            │                              ExposureControl
            ▼                              JpegEncoder
    PipelineThread                         BufferProcessor
    ↳ acquireSlot, demosaic compute,
      per-output blits, single submit
            │
            │     ┌──────── DMABUF / OPAQUE_FD ────────┐
            ▼     ▼                                    │
    VulkanIspPipeline                    CaptureThread / V4l2Source
    ┌─────────────────────────┐                  │
    │ compute: Bayer→scratch  │←──────  DQBUF, drain-to-latest
    │ fragment: scratch→gralloc│                 │
    │ release_fence per output│       /dev/video0 + /dev/v4l-subdev*
    └─────────┬──────┬────────┘
              │      │
              ▼      ▼
        gralloc    JpegSnapshot (host-mapped)
                          │
                          ▼
                    JpegWorker (libjpeg async, EXIF orientation)
                          │
                          ▼
                    ResultThread → process_capture_result

    StatsWorker (NEON, parallel to GPU): rgbMean[16×16] +
        focusMetric[ROI] → IPA (BasicIpa) → DelayedControls
```

Per-frame control writes (exposure, gain) round-trip through
`DelayedControls` so they land at `frameNumber + controlDelay[id]`
where the silicon actually applies them. WB gains and CCM apply in
the demosaic shader (zero silicon delay).

The full prose architecture used to live in
[`docs/architecture.md`](docs/architecture.md), but parts of that
document predate the Tier 3 async-pipeline rewrite and the HAL3.4
contract pass. For current shape consult
[`docs/tier3_architecture.md`](docs/tier3_architecture.md) and the
section above. Developer notes for individual subsystems live under
[`docs/`](docs/).

## Build & deploy

Standard AOSP module. From an AOSP 7.1 / LineageOS 14.1 tree:

```bash
# Add to your device makefile
PRODUCT_PACKAGES += camera.$(TARGET_BOARD_PLATFORM)

# Build (JDK 8 needed for the AOSP 7.1 host tools)
mmm hardware/camera
```

Outputs:
- `out/.../system/lib/hw/camera.tegra.so` — the HAL.
- `out/.../system/vendor/etc/camera/tuning/<sensor>_<integrator>.json`
  — tuning data, installed via `BUILD_PREBUILT`.

Both must land on the device under `/system/lib/hw/` and
`/system/vendor/etc/camera/tuning/` respectively, then `cameraserver`
needs a restart.

## What's not implemented

- **RAW / DNG output** — adding `RAW16` / `RAW_OPAQUE` would mandate
  `BLACK_LEVEL_PATTERN`, `WHITE_LEVEL`, `COLOR_TRANSFORM_*`,
  `FORWARD_MATRIX_*`, `CALIBRATION_TRANSFORM_*`, `NOISE_PROFILE`,
  plus a RAW16 stream producer. Not pursued — the Mi Pad 1 use case
  doesn't need DNG.
- **Reprocessing / ZSL** (`PRIVATE_REPROCESSING`,
  `YUV_REPROCESSING`). The `Request::inputBuffer` slot is reserved
  but not wired.
- **`MANUAL_SENSOR`** — would need pre-capture-trigger contract +
  per-frame sensor settings round-trip we don't implement (manual
  exposure / gain via `SENSOR_EXPOSURE_TIME` / `SENSOR_SENSITIVITY`
  in non-OFF AE mode is honoured opportunistically through
  `BasicIpa`'s pull, but the strict MANUAL_SENSOR contract isn't
  claimed).
- **`CONSTRAINED_HIGH_SPEED_VIDEO`** — would need a high-fps sensor
  mode and a relaxed-metadata path the HAL doesn't have.
- **`DEPTH_OUTPUT`** — no depth backing.
- Output above 1920×1080 — SW ISP can't sustain the rate.
- Packed-YUV sensors (UYVY / YUY2) — Bayer-only.

## Hardware quirks the code is built around

- **Tegra K1 single-queue Vulkan** — submits serialise on the GPU
  side. The cmd-buffer / fence / scratch ring of depth 2 is sized
  for CPU↔GPU overlap, not parallel GPU execution.
- **Compute-store to a `VK_ANDROID_native_buffer` image produces
  swizzled garbage on this driver.** Fragment ROP is the only
  correct write path. `vkCmdCopyImage` / `CopyBufferToImage`
  targeting a gralloc image likewise miss the tiler.
- **`VK_ANDROID_native_buffer` is filtered out by `libvulkan.so` for
  app-level callers** but exposed to HAL processes. We bypass
  `libvulkan.so` and dispatch through a HAL-direct PFN table loaded
  from `libglcore.so` so the extension is reachable.
- **`vkGetMemoryFdPropertiesKHR` on external gralloc fds returns
  `VK_ERROR_INITIALIZATION_FAILED` with `typeBits=0`.** Gralloc
  zero-copy import as a Vulkan image is impossible — the round trip
  through fragment ROP is mandatory.

## Origin

Forked from Antmicro's V4L2-based Android Camera HAL3 reference. The
top-level Camera3 ops table and V4L2 wiring trace back to that fork;
everything that runs after the V4L2 buffer is dequeued — ISP, 3A,
async pipeline, multi-stream, JPEG, Camera2 metadata surface — has
been written from scratch.
