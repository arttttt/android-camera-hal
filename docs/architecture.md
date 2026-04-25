# Architecture

Targets the Xiaomi Mi Pad 1 (Tegra K1, codename `mocha`) running
LineageOS 14.1 / Android 7.1.2. Bayer-only Vulkan ISP is the one and
only data path; the HAL has no CPU fallback.

## Component overview

```
                      Android camera framework
                               │
                     ┌─────────▼─────────┐
                     │    CameraModule   │  (hal/HalModule.cpp)
                     │   (HAL module)    │
                     └─────────┬─────────┘
                               │ opens per-camera
                     ┌─────────▼─────────┐
                     │      Camera       │  (hal/Camera.cpp)
                     │  (Camera3 device) │
                     └──┬───────────┬────┘
                        │           │
              ┌─────────▼───┐   ┌───▼────────────┐
              │  V4l2Device │   │ VulkanIsp      │
              │(/dev/video0)│   │ Pipeline       │
              └──────┬──────┘   │ (compute +     │
                     │          │  fragment)     │
                     │          └───┬────────────┘
                     ▼              │
              /dev/v4l-subdev*      │
              (focuser / VCM)       │
                                    ▼
                             gralloc (preview)
                             libjpeg (BLOB)
```

`V4l2Device` hands `VulkanIspPipeline`-exported dma-buf fds to
`VIDIOC_QBUF` so the VI DMA writes captured Bayer directly into the
GPU-visible input ring — no CPU copy on the hot path. See
[isp-pipeline.md](isp-pipeline.md) for the full ISP flow.

The main objects:

- **`Camera`** (`hal/Camera.cpp`, `hal/Camera.h`) — one instance per physical
  camera. Implements the Camera3 device ops (`process_capture_request`,
  `configure_streams`, `construct_default_request_settings`, …). Feature
  logic lives in sub-packages under `hal/`:
  - `hal/3a/` — per-frame AE/AF controllers (`ExposureControl`,
    `AutoFocusController`).
  - `hal/metadata/` — stateless builders for static characteristics,
    per-template request defaults, per-frame result echoes
    (`CameraStaticMetadata`, `RequestTemplateBuilder`,
    `ResultMetadataBuilder`).
  - `hal/jpeg/` — `JpegEncoder` (BLOB path).
  - `hal/pipeline/` — `StreamConfig` (stream-list normalisation +
    V4L2 resolution pick) and `BufferProcessor` (per-output-buffer
    zero-copy / BLOB dispatch).

- **`V4l2Device`** (`v4l2/V4l2Device.cpp/.h`) — thin C++ wrapper over
  `/dev/video0`. Speaks both `V4L2_MEMORY_MMAP` and `V4L2_MEMORY_DMABUF`;
  `setDmaBufFds()` switches into DMABUF mode with caller-supplied
  capture fds. Manages `VIDIOC_QBUF`/`VIDIOC_DQBUF`, `VIDIOC_S_CTRL`
  for sensor controls. Deferred-QBUF on unlock (DMABUF mode only) so
  V4L2 never reuses a slot the shader is still reading. Also opens
  the focuser subdev (`/dev/v4l-subdev*`) when found. The `Resolution`
  struct used across V4L2 / streams / metadata lives in its own
  header (`v4l2/Resolution.h`).

- **`IspPipeline`** (`isp/IspPipeline.h`) — abstract base with one
  concrete backend (`VulkanIspPipeline`). The interface is deliberately
  narrow: `processToGralloc` (zero-copy RGBA blit) + `processToCpu`
  (synchronous demosaic into a CPU-mapped VkBuffer for the JPEG path) +
  `prewarm` + the DMABUF input-ring helpers. See
  [isp-pipeline.md](isp-pipeline.md) for the Vulkan path detail.

- **Debug helpers** (`util/`) — `AutoLogCall.h`, `FpsCounter.h`,
  `Benchmark.h` hold the three per-call/per-frame instrumentation
  classes (each used via a `DBGUTILS_*` / `FPSCOUNTER_*` / `BENCHMARK_*`
  macro). `util/DbgUtils.h` is a thin facade including all three.

## Request lifecycle

All Camera3 per-frame work happens in `Camera::processCaptureRequest()`.
The flow is **strictly synchronous**, single-threaded, single-buffer:

```
1. mExposure->onSettings(cm)            ← parse + apply exposure/gain/EV comp
                                          via VIDIOC_S_CTRL
2. mAf->onSettings(cm, frame_number)    ← AF mode / trigger / continuous
                                          state machine (AutoFocusController)
3. notifyShutter(frame_number, timestamp)
4. mIsp->waitForPreviousFrame()         ← drain prev GPU work so V4L2
                                          can reuse the input slot
5. mDev->readLock()                     ← DQBUF one V4L2 buffer (blocking);
                                          also flushes deferred QBUFs
6. mAf->onFrameStart()                  ← step VCM if a sweep is in flight
7. Parse zoom crop region
8. for each output buffer:
     mBufferProcessor->processOne(...)  ← per-buffer dispatch:
       - Wait acquire fence
       - RGBA_8888 → processToGralloc: GPU demosaic + crop/scale + blit
         to gralloc, submits async, returns a release_fence fd.
       - BLOB     → SW_WRITE_OFTEN lock + JpegEncoder::encode
                    (synchronous processToCpu → libjpeg).
9.  mDev->unlock(frame)                 ← in DMABUF mode: stash slot
                                          for deferred QBUF at step 5
                                          of the next frame
10. ResultMetadataBuilder::build(cm, …) ← per-frame echo metadata
11. callbacks.process_capture_result(result)
```

The entire sequence runs under `mMutex` (held from step 1 through step 10).
Framework may have queued N capture requests ahead, but they are dispatched
one at a time.

There is **no CPU RGBA fallback**: a `processToGralloc` failure is a
hardware / driver error and propagates as `NO_INIT`. Packed-YUV sensors
(UYVY / YUYV) are also not supported — the target hardware is Bayer
only.

## Stream configuration

`Camera::configureStreams()` receives the set of streams the framework wants
(preview, still, video, JPEG thumbnail, …). The HAL:

1. `StreamConfig::normalize()` validates and rewrites the stream list:
   - Rejects ZSL usage and multiple input streams (BAD_VALUE).
   - Remaps `HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED` → `RGBA_8888`.
   - Rewrites `usage` per stream_type (SW_WRITE_OFTEN / SW_READ_OFTEN).
   - Sets `max_buffers` (matches `V4L2DEVICE_BUF_COUNT`).
   - Picks the V4L2 capture resolution: an `HW_VIDEO_ENCODER` stream
     wins (so the sensor locks to the matching FPS mode), else the
     largest non-BLOB stream, else the largest BLOB.
2. Creates the Vulkan ISP, prewarms it (allocates the Vulkan input
   ring at target size) and exports each slot as an OPAQUE_FD dma-buf.
3. Calls `V4l2Device::setDmaBufFds(fds, N)` to switch V4L2 into
   `V4L2_MEMORY_DMABUF` mode, then `setResolution()` →
   `VIDIOC_S_FMT` + `REQBUFS(DMABUF)` + `VIDIOC_QBUF(.m.fd=fds[i])`.
   If export fails the code silently stays in MMAP + memcpy mode.
4. Creates the per-camera helpers (`AutoFocusController`,
   `ExposureControl`, `JpegEncoder`, `BufferProcessor`) with the new
   ISP handle.

No per-stream output-buffer allocation happens in the HAL — gralloc
buffers come from the framework. The HAL owns the V4L2 capture
buffers (MMAP) or, in DMABUF mode, the Vulkan input ring (exported
to V4L2 via fds).

## Threading

- **Framework thread** — calls `processCaptureRequest`. This is where all
  host work happens.
- **GPU queue** — the Vulkan queue runs compute demosaic + fragment
  blit asynchronously with respect to the framework thread. Completion
  is communicated via a sync_fence fd returned as the gralloc
  buffer's `release_fence`; the framework composites once the fence
  signals.
- **Camera3 callback thread** — the framework's thread on which we
  invoke `notify` and `process_capture_result`. Currently we call these
  from the framework thread itself (tail of `processCaptureRequest`),
  so there is no separate callback thread in practice.

There is **no dedicated capture / ISP thread**. This is the root cause of
the pipeline-depth-1 behaviour discussed in
[latency-and-buffers.md](latency-and-buffers.md).

## Static characteristics

Built once per camera by `CameraStaticMetadata::build()` in
`hal/metadata/`. The keys populated today are enumerated in
[camera3-compliance.md](camera3-compliance.md) along with the gaps.
Values are a mix of sensor-derived (resolutions, sensor area from
`SensorConfig`), hardcoded (optical properties — 3.3 mm focal length,
1.8 mm physical sensor width), and inferred (VCM range from the focuser
subdev via `VIDIOC_QUERYCTRL`). `Camera` caches the result of the first
call.

## AF state machine

Owned by `AutoFocusController` in `hal/3a/`. Camera dispatches into it
at three per-frame points (`onSettings`, `onFrameStart`,
`onSharpnessStats`) and reads back AF/focus state via `report()`.
Semantics:

- **AF_MODE_OFF** — pass through `LENS_FOCUS_DISTANCE` verbatim to VCM.
  Trigger is a no-op.
- **AF_MODE_AUTO** / **MACRO** — one-shot contrast sweep on trigger:
  steps VCM across the range, settle frames per step (from tuning),
  picks the position with the highest Tenengrad score over the centre
  8x8 patches of the IPA stats grid. AWB is locked during the sweep
  (via `IspPipeline::setAwbLock(true)`) to prevent exposure drift from
  polluting the metric.
- **AF_MODE_CONTINUOUS_PICTURE** — re-triggers a sweep every 60 frames.

The sharpness grid comes from `IpaStats::sharpness[16][16]` produced by
`NeonStatsEncoder` off the raw Bayer green channel — same buffer the
IPA's AE/AWB consumes — so AF no longer locks the rendered RGBA
gralloc for a CPU read.

## Configuration knobs

Compile-time, defined in `Android.mk`:

- `V4L2DEVICE_BUF_COUNT` — V4L2 buffer count. Currently `4`.
  See [latency-and-buffers.md](latency-and-buffers.md) for context on
  why this value matters.
- `V4L2DEVICE_OPEN_ONCE` — open `/dev/video0` once at HAL start vs.
  per `open()` call.

Runtime knobs via setprop (`hal/Camera.cpp` parses `persist.camera.*`):

- `persist.camera.soft_isp` — `0` / `1`. `1` (default) enables the
  Vulkan ISP and the 3A controllers. `0` disables 3A and leaves
  exposure/gain under whatever the sensor kernel driver defaults to;
  kept as a fallback while porting but no longer exercised on the
  production path.

## Per-module tuning files

`/vendor/etc/camera/tuning/<lower(sensor)>_<lower(integrator)>.json`
— converted from the stock NVIDIA `.isp` overrides via
`tools/isp_to_json.py`. The JSON splits keys into two sections:

- `active` — paths with a live HAL consumer today:
  `af.*` (VCM), `colorCorrection.Set[]` (multi-CCT CCM),
  `opticalBlack.*` (black-level subtract), `mwbCCT.*` (deferred),
  plus a `module` block with per-module datasheet values that are
  not in the `.isp` (physical size, focal length, min focus
  distance).
- `reserved` — preserved 1:1 from the stock tuning: noise reduction
  v2/v6, lens shading, tone curves, sharpness filters, full AE VFR,
  flicker detection, AWB LUTs. Consumed as the corresponding shader
  stages ship — no schema migration required for that promotion.

`SensorTuning` (in `isp/sensor/`) loads at Camera construction time,
`!isLoaded()` falls back to compile-time defaults so the HAL stays
alive even with the tuning dir missing.

The `.isp → .json` converter is not part of the build; it's run once
per stock blob drop and the output committed under `tuning/`. See
`docs/open-questions.md` for CCM / wbGain consumption paths blocked
on Tier 3 AWB.
