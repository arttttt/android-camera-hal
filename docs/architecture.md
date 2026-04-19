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

- **`Camera`** (`hal/Camera.cpp`, `hal/Camera.h`) — one instance per physical camera.
  Implements the Camera3 device ops (`process_capture_request`,
  `configure_streams`, `construct_default_request_settings`, …). Holds
  per-camera state: default request templates, static characteristics,
  cached `V4l2Device` pointer, `IspPipeline*`, AF state machine.

- **`V4l2Device`** (`v4l2/V4l2Device.cpp/.h`) — thin C++ wrapper over
  `/dev/video0`. Speaks both `V4L2_MEMORY_MMAP` and `V4L2_MEMORY_DMABUF`;
  `setDmaBufFds()` switches into DMABUF mode with caller-supplied
  capture fds. Manages `VIDIOC_QBUF`/`VIDIOC_DQBUF`, `VIDIOC_S_CTRL`
  for sensor controls. Deferred-QBUF on unlock (DMABUF mode only) so
  V4L2 never reuses a slot the shader is still reading. Also opens
  the focuser subdev (`/dev/v4l-subdev*`) when found.

- **`IspPipeline`** (`isp/IspPipeline.h`) — abstract base for demosaic
  / colour processing. See [isp-pipeline.md](isp-pipeline.md) for the
  two live backends.

- **`ImageConverter`** (`image/ImageConverter.cpp`) — libyuv-backed YUV/UYVY/YUYV →
  RGBA / JPEG paths, used when the sensor emits packed YUV (no ISP needed).

- **`Workers`** (`util/Workers.cpp/h`) — a generic thread pool. Currently used
  only by `CpuIspPipeline` / `ImageConverter` for parallelising row-wise
  work. **Not** used for request pipelining.

## Request lifecycle

All Camera3 per-frame work happens in `Camera::processCaptureRequest()`
(`hal/Camera.cpp:741`). The flow is **strictly synchronous**, single-threaded,
single-buffer:

```
1. Parse request settings (exposure, gain, AF trigger, zoom, JPEG quality)
2. Apply settings via VIDIOC_S_CTRL (exposure, gain, frame length)
3. Handle AF trigger state machine (start/cancel sweep)
4. notifyShutter(frame_number, timestamp)
5. mIsp->waitForPreviousFrame()         ← drain prev GPU work so V4L2
                                          can reuse the input slot
6. mDev->readLock()                     ← DQBUF one V4L2 buffer (blocking);
                                          also flushes deferred QBUFs
7. for each output buffer:
   a. Wait acquire fence
   b. RGBA: mIsp->processToGralloc(src_slot=frame->index, …) — GPU
            submits async, returns a release_fence fd for the buffer
   c. BLOB: mIsp->processSync() into mRgbaTemp, then JPEG encode
8. AF sweep: evaluate sharpness, move VCM, update state
9. mDev->unlock(frame)                  ← in DMABUF mode: stash slot
                                          for deferred QBUF at step 6
                                          of the next frame
10. Build result metadata
11. callbacks.process_capture_result(result)
```

The entire sequence runs under `mMutex` (held from step 1 through step 10).
Framework may have queued N capture requests ahead, but they are dispatched
one at a time.

## Stream configuration

`Camera::configureStreams()` receives the set of streams the framework wants
(preview, still, video, JPEG thumbnail, …). The HAL:

1. Picks a single V4L2 resolution to request from the sensor — typically
   the largest stream's resolution, clamped to supported sensor modes.
2. Creates the soft-ISP backend, prewarms it (allocates the Vulkan
   input ring at target size) and exports each slot as an OPAQUE_FD
   dma-buf.
3. Calls `V4l2Device::setDmaBufFds(fds, N)` to switch V4L2 into
   `V4L2_MEMORY_DMABUF` mode, then `setResolution()` →
   `VIDIOC_S_FMT` + `REQBUFS(DMABUF)` + `VIDIOC_QBUF(.m.fd=fds[i])`.
   If export fails the code silently stays in MMAP + memcpy mode.
4. Remaps `HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED` → `RGBA_8888`.
5. Rejects ZSL and input streams.
6. Sets `max_buffers = 4` on every output stream (matches
   `V4L2DEVICE_BUF_COUNT`).

No per-stream output-buffer allocation happens in the HAL — gralloc
buffers come from the framework. The HAL owns the V4L2 capture
buffers (MMAP) or, in DMABUF mode, the Vulkan input ring (exported
to V4L2 via fds).

## Threading

- **Framework thread** — calls `processCaptureRequest`. This is where all
  real work happens.
- **Worker pool** (`Workers::gWorkers`) — CPU worker threads for
  parallel rows in `ImageConverter` (UYVY/YUYV paths). Synchronous
  fork/join: framework thread blocks until workers are done.
- **Camera3 callback thread** — the framework's thread on which we
  invoke `notify` and `process_capture_result`. Currently we call these
  from the framework thread itself (tail of `processCaptureRequest`),
  so there is no separate callback thread in practice.

There is **no dedicated capture / ISP thread**. This is the root cause of
the pipeline-depth-1 behaviour discussed in
[latency-and-buffers.md](latency-and-buffers.md).

## Static characteristics

Built once per camera in `Camera::getStaticCharacteristics()`
(`hal/Camera.cpp:145`). The keys populated today are enumerated in
[camera3-compliance.md](camera3-compliance.md) along with the gaps. Values
are a mix of sensor-derived (resolutions, sensor area from `SensorConfig`),
hardcoded (optical properties — 3.3 mm focal length, 1.8 mm physical
sensor width), and inferred (VCM range from the focuser subdev via
`VIDIOC_QUERYCTRL`).

## AF state machine

Live in `hal/Camera.cpp` across `mAfSweep*`, `mAfSettleFrames`, `mFocusPosition`.
Semantics:

- **AF_MODE_OFF** — pass through `LENS_FOCUS_DISTANCE` verbatim to VCM.
- **AF_MODE_AUTO** / **MACRO** — one-shot contrast sweep on trigger:
  steps VCM across the range, 2-frame settle per step, picks the position
  with the highest normalised Laplacian score in the centre 1/4 of the
  frame. AWB is locked during the sweep to prevent exposure drift from
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
