# Architecture

## Component overview

```
                      Android camera framework
                               │
                     ┌─────────▼─────────┐
                     │    CameraModule   │  (CameraModule.cpp)
                     │   (HAL module)    │
                     └─────────┬─────────┘
                               │ opens per-camera
                     ┌─────────▼─────────┐
                     │      Camera       │  (hal/Camera.cpp)
                     │  (Camera3 device) │
                     └──┬───────────┬────┘
                        │           │
              ┌─────────▼───┐   ┌───▼────────────┐
              │  V4l2Device │   │   IspPipeline  │
              │(/dev/video0)│   │  (interface)   │
              └──────┬──────┘   └───┬────────────┘
                     │              │
                     │        ┌─────┴──────┐
                     │        │            │
                     │   ┌────▼──────┐ ┌───▼────────────┐
                     │   │ VulkanIsp │ │     HwIsp      │
                     │   │(compute + │ │(libnvisp_v3)   │
                     │   │ fragment) │ │                │
                     │   └───────────┘ └────────────────┘
                     ▼
              /dev/v4l-subdev* (focuser/VCM)
```

`V4l2Device` hands `VulkanIspPipeline`-exported dma-buf fds to
`VIDIOC_QBUF` so the VI DMA writes captured Bayer directly into the
GPU-visible input ring — no CPU copy on the hot path. See
[isp-pipeline.md](isp-pipeline.md) for the full ISP flow.

The main objects:

- **`Camera`** (`hal/Camera.cpp`, `hal/Camera.h`) — one instance per physical
  camera. Implements the Camera3 device ops (`process_capture_request`,
  `configure_streams`, `construct_default_request_settings`, …). After
  the Tier 1.1 refactor it is a thin dispatcher; feature logic lives in
  sub-packages under `hal/`:
  - `hal/3a/` — per-frame AE/AF controllers (`ExposureControl`,
    `AutoFocusController`).
  - `hal/metadata/` — stateless builders for static characteristics,
    per-template request defaults, per-frame result echoes
    (`CameraStaticMetadata`, `RequestTemplateBuilder`,
    `ResultMetadataBuilder`).
  - `hal/jpeg/` — `JpegEncoder` (BLOB path).
  - `hal/pipeline/` — `StreamConfig` (stream-list normalisation +
    V4L2 resolution pick) and `BufferProcessor` (per-output-buffer
    zero-copy / CPU-fallback logic).

- **`V4l2Device`** (`v4l2/V4l2Device.cpp/.h`) — thin C++ wrapper over
  `/dev/video0`. Speaks both `V4L2_MEMORY_MMAP` and `V4L2_MEMORY_DMABUF`;
  `setDmaBufFds()` switches into DMABUF mode with caller-supplied
  capture fds. Manages `VIDIOC_QBUF`/`VIDIOC_DQBUF`, `VIDIOC_S_CTRL`
  for sensor controls. Deferred-QBUF on unlock (DMABUF mode only) so
  V4L2 never reuses a slot the shader is still reading. Also opens
  the focuser subdev (`/dev/v4l-subdev*`) when found. The `Resolution`
  struct used across V4L2 / streams / metadata lives in its own
  header (`v4l2/Resolution.h`).

- **`IspPipeline`** (`isp/IspPipeline.h`) — abstract base for demosaic
  / colour processing. See [isp-pipeline.md](isp-pipeline.md) for the
  two live backends.

- **`ImageConverter`** (`image/ImageConverter.cpp`) — libyuv-backed
  UYVY/YUY2 → RGBA / JPEG paths, used when the sensor emits packed YUV
  (no ISP needed). Dies in Tier 1.5 along with the rest of the CPU
  fallback.

- **`Workers`** (`util/Workers.cpp/h`) — a generic thread pool. Currently used
  only by `ImageConverter` for parallelising row-wise work. **Not**
  used for request pipelining.

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
       - Zero-copy: processToGralloc(src_slot=frame->index, …) — GPU
         submits async, returns a release_fence fd for the buffer
       - CPU fallback (RGBA): demosaic → zoom-crop into gralloc
       - CPU fallback (BLOB): JpegEncoder::encode
9. mAf->onFrameData(rgba, …)            ← contrast metric for the sweep
10. mDev->unlock(frame)                 ← in DMABUF mode: stash slot
                                          for deferred QBUF at step 5
                                          of the next frame
11. ResultMetadataBuilder::build(cm, …) ← per-frame echo metadata
12. callbacks.process_capture_result(result)
```

The entire sequence runs under `mMutex` (held from step 1 through step 10).
Framework may have queued N capture requests ahead, but they are dispatched
one at a time.

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
2. Creates the soft-ISP backend, prewarms it (allocates the Vulkan
   input ring at target size) and exports each slot as an OPAQUE_FD
   dma-buf.
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
  real work happens.
- **Worker pool** (`Workers::gWorkers`) — CPU worker threads for
  parallel rows in `ImageConverter` (UYVY/YUY2 paths). Synchronous
  fork/join: framework thread blocks until workers are done.
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
at three per-frame points (`onSettings`, `onFrameStart`, `onFrameData`)
and reads back AF/focus state via `report()`. Semantics:

- **AF_MODE_OFF** — pass through `LENS_FOCUS_DISTANCE` verbatim to VCM.
  Trigger is a no-op.
- **AF_MODE_AUTO** / **MACRO** — one-shot contrast sweep on trigger:
  steps VCM across the range, 2-frame settle per step, picks the position
  with the highest normalised Laplacian score in the centre 1/4 of the
  frame. AWB is locked during the sweep (via
  `IspPipeline::setAwbLock(true)`) to prevent exposure drift from
  polluting the metric.
- **AF_MODE_CONTINUOUS_PICTURE** — re-triggers a sweep every 60 frames.

The sweep reads **the rendered preview RGBA buffer**, not a dedicated
statistics channel. This is a known limitation — see
[camera3-compliance.md](camera3-compliance.md).

## Configuration knobs

Compile-time, defined in `Android.mk`:

- `V4L2DEVICE_BUF_COUNT` — V4L2 mmap buffer count. Currently `4`.
  See [latency-and-buffers.md](latency-and-buffers.md) for context on
  why this value matters.
- `V4L2DEVICE_FPS_LIMIT` — soft fps cap via `usleep` between DQBUFs.
  Currently `0` (disabled).
- `V4L2DEVICE_OPEN_ONCE` — open `/dev/video0` once at HAL start vs.
  per `open()` call.
- `V4L2DEVICE_USE_POLL` — use `poll()` before `DQBUF` (timeout 5 s).

Runtime knobs via setprop (`hal/Camera.cpp` parses `ro.hal.camera.*`):

- `soft_isp` — `0`/`1` toggle between hardware ISP (Tegra `libnvisp_v3`)
  and software pipeline.
