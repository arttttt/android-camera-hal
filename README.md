# Tegra K1 / Mi Pad 1 Camera HAL

V4L2-based Android Camera HAL for **Xiaomi Mi Pad 1** (codename
`mocha`, Tegra K1, single-SMX Kepler GK20A) running LineageOS 19.1 /
Android 12. Software ISP runs on the GPU via Vulkan compute +
fragment, 3A statistics on ARMv7 NEON. No closed NVIDIA blob (no
`libnvodm_imager` / `libnvcamerautils`); all processing is ours from
the raw Bayer pixel onward.

## Status

| Property | Value |
|----------|-------|
| HAL device API | `CAMERA_DEVICE_API_VERSION_3_4` |
| Module API | `CAMERA_MODULE_API_VERSION_2_4` (no flash: `set_torch_mode` → `-ENOSYS`) |
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
- **Asynchronous request pipeline.** Six engines per camera, each
  one thread over a job queue, polling fds instead of blocking:
  - *Capture* — applies the request's settings, stamps the shutter,
    takes the newest Bayer (drain-to-latest, so preview never reaches
    into a stale frame); owns the V4L2 queue.
    `process_capture_request` returns in < 1 ms.
  - *Stats* — NEON 3A statistics over the raw Bayer slot, progressive
    across two frames to keep peak CPU low; always the freshest frame.
  - *ISP* — records and submits the GPU work, polls its fence (no
    `vkWaitForFences` on the hot path).
  - *Finish* — the CPU half: YUV finalisation, 3A, partial results.
  - *JPEG* — libjpeg encode off the GPU's path, from a snapshot ring
    of host-mapped Vulkan buffers.
  - *Result* — hands frames back strictly in request order, each once
    no job holds it any more, so async JPEG cannot reorder results.

  Capture slots, JPEG snapshots and frames are owned by move-only
  types (lease, permit, token) that give them back on every path, so
  flush, stop and errors cannot leak one. See
  [`docs/tier3_architecture.md`](docs/tier3_architecture.md).
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
- **AWB** — picked per-sensor by `AwbFactory::createAwb` from the
  tuning's `active.awb.algorithm` flag.
  *IMX179 (rear)* runs RPi-style **Bayesian AWB**: coarseSearch
  over CT curves (calibrated post-OB raw R/G and B/G ratios per
  illuminant) with a lux-conditioned log-prior, deltaLimit
  per-zone error clamp, biasCT pull toward a fallback CT, and
  off-curve fineSearch in (R/G, B/G) chromaticity space for
  magenta ↔ green refinement. IIR damping smooths published gains
  after a startup-snap window. Manual AWB presets
  (INCANDESCENT / DAYLIGHT / etc.) clip the coarseSearch range
  to per-mode CT windows; matching modes are advertised in
  `ANDROID_CONTROL_AWB_AVAILABLE_MODES`.
  *OV5693 (front)* stays on the original gray-world fallback —
  patch-grid average + 96-valid-patch confidence gate +
  EMA-relax to a calibrated daylight prior — until proper
  grey-card calibration lands.
  Both share the downstream pipe: WB applied in the demosaic
  shader (zero silicon delay), CCM LERP'd across `colorCorrection
  .Set[]` and updated through the shared Q10 buffer.
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
  `get_metadata_vendor_tag_ops` are NULL'd.
- Partial results: `PARTIAL_RESULT_COUNT = 3` — AWB state, then AE
  state, as soon as 3A has run on the frame; the full result last.
- AE / AWB / AF state reported per-frame in result metadata.

### Tuning

- Per-module JSON at `/vendor/etc/camera/tuning/<sensor>_<integrator>.json`.
- Source: stock NVIDIA `.isp` files (Mi Pad 1 vendor blob) converted
  by `tools/isp_to_json.py`, split into an `active` section (paths
  the HAL consumes) and a `reserved` section (preserved verbatim for
  future stages). Module facts (focal length, min focus distance,
  bayer pattern, sensor orientation) live in
  `tuning/_module_<sensor>_<integrator>.json` and are merged at
  conversion time.
- HAL-specific knobs that don't exist in NVIDIA `.isp` (the AE
  close-speed-zone width and the smallest area an AE region meters
  over, per sensor; the contrast-AF knobs) live under an
  `active.hal_overrides` sub-section in the same per-module JSON,
  carved out so `tools/isp_to_json.py` preserves them verbatim across
  regenerations from the upstream `.isp`.
- Live consumers: `AutoFocusController`, `SensorTuning::ccmForCctQ10`,
  `DemosaicCompute` shader (optical-black bias), `Ipa3A` AE+AWB,
  `CameraStaticMetadata` (per-module geometry).
- Reserved (not yet consumed): lens shading, full AE VFRTable, full
  AWB CCT LUT, noise reduction sets, tone curves, sharpness filters.

### Output formats and stream sizing

- Advertised resolution set is filtered to **16:9 with a 1920×1080
  ceiling** — the ceiling tracks what the SW ISP can sustain at
  frame rate on this GPU. Below the ceiling every V4L2-supported
  16:9 mode is exposed.
- Auxiliary RGBA streams the framework configures at non-advertised
  sizes (e.g. face-detect inputs) are accepted up to the same
  dimension cap regardless of aspect. A YUV stream must be at the
  capture size: the NV12 path does not scale.
- `IMPLEMENTATION_DEFINED` resolves by usage flags:
  `HW_VIDEO_ENCODER` → `YCbCr_420_888`, everything else →
  `RGBA_8888`. NV12 / I420 / YV12 chroma layouts are repacked from
  the NV12 the GPU produces; NV21 returns `NO_INIT`.

## Architecture (high level)

```
                    Android camera provider (HIDL 2.4)
                                  │ camera3_device ops
   hal/ camera.tegra  ┌───────────▼────────────┐
   Camera3 frontend   │ Camera3Device          │ metadata ⇄ FrameSettings,
                      │ Camera3Request         │ static metadata from
                      │ Camera3ResultSink      │ Capability, results from
                      │ Camera3GrallocPort     │ Core3AResult
                      └───────────┬────────────┘
                                  │ mochacamera/*.h: Session, ports
   core/ libmochacamera ┌─────────▼──────────┐
   camera core          │ Session            │
                        └─────────┬──────────┘
        submit ─► Capture ─► ISP (GPU) ─► Finish ─► JPEG ─► Result ─► sink
                    │  ▲                    ▲ 3A
                    │  └ V4L2 (DMABUF into  │
                    │    Vulkan input ring) │
                    └──► Stats (NEON) ──────┘
```

The core knows nothing of Camera3: no `camera3.h`, no
`CameraMetadata`. It takes requests as `FrameSettings`, describes
the camera as a `Capability`, and reports through a `ResultSink`;
the frontend translates both ways. Per-frame control writes
(exposure, gain) round-trip through `DelayedControls` so they land at
`frameNumber + controlDelay[id]`, where the silicon applies them. WB
gains and CCM apply in the demosaic shader (zero silicon delay).

- [`docs/architecture.md`](docs/architecture.md) — modules, the
  boundary, request lifecycle, stream configuration, 3A, tuning.
- [`docs/tier3_architecture.md`](docs/tier3_architecture.md) — the
  engines, frame ownership, flush / stop / errors.

Developer notes for individual subsystems live under
[`docs/`](docs/).

## Build & deploy

`Android.bp` module, from a LineageOS 19.1 tree:

```bash
# Add to your device makefile
PRODUCT_PACKAGES += camera.tegra

mmm hardware/camera
```

Outputs (vendor partition):
- `vendor/lib/hw/camera.tegra.so` — the HAL (`libmochacamera`, the
  core, is linked in statically).
- `vendor/etc/camera/tuning/<sensor>_<integrator>.json` — tuning
  data, from the `prebuilt_etc` modules.

After replacing them on a device, restart the provider:
`setprop ctl.restart vendor.camera-provider-2-4`.

The core's host tests run under ThreadSanitizer from the repository
root: `core/tests/run_host_tests.sh`.

## What's not implemented

- **RAW / DNG output** — adding `RAW16` / `RAW_OPAQUE` would mandate
  `BLACK_LEVEL_PATTERN`, `WHITE_LEVEL`, `COLOR_TRANSFORM_*`,
  `FORWARD_MATRIX_*`, `CALIBRATION_TRANSFORM_*`, `NOISE_PROFILE`,
  plus a RAW16 stream producer. Not pursued — the Mi Pad 1 use case
  doesn't need DNG.
- **Reprocessing / ZSL** (`PRIVATE_REPROCESSING`,
  `YUV_REPROCESSING`). An input stream configures, but a request's
  input buffer is not read.
- **`MANUAL_SENSOR`** — would need pre-capture-trigger contract +
  per-frame sensor settings round-trip we don't implement (manual
  exposure / gain via `SENSOR_EXPOSURE_TIME` / `SENSOR_SENSITIVITY`
  in non-OFF AE mode is honoured opportunistically through
  `Ipa3A`'s pull, but the strict MANUAL_SENSOR contract isn't
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
