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
                     │      Camera       │  (Camera.cpp)
                     │  (Camera3 device) │
                     └──┬───────────┬────┘
                        │           │
              ┌─────────▼───┐   ┌───▼────────────┐
              │  V4l2Device │   │   IspPipeline  │
              │ (/dev/video0)│   │  (interface)   │
              └──────┬──────┘   └───┬────────────┘
                     │              │
                     │    ┌─────────┴──────────┬──────────────┬──────────────┐
                     │    │                    │              │              │
                     │ ┌──▼─────┐        ┌─────▼───┐    ┌─────▼────┐  ┌──────▼─────┐
                     │ │CpuIsp  │        │GlesIsp  │    │VulkanIsp │  │  HwIsp     │
                     │ │(libyuv)│        │(shaders)│    │(compute) │  │(libnvisp_v3)│
                     │ └────────┘        └─────────┘    └──────────┘  └────────────┘
                     │
                     ▼
              /dev/v4l-subdev* (focuser/VCM)
```

The main objects:

- **`Camera`** (`Camera.cpp`, `Camera.h`) — one instance per physical camera.
  Implements the Camera3 device ops (`process_capture_request`,
  `configure_streams`, `construct_default_request_settings`, …). Holds
  per-camera state: default request templates, static characteristics,
  cached `V4l2Device` pointer, `IspPipeline*`, AF state machine.

- **`V4l2Device`** (`V4l2Device.cpp/.h`) — thin C++ wrapper over
  `/dev/video0`. Manages mmap buffers, `VIDIOC_QBUF`/`VIDIOC_DQBUF`,
  `VIDIOC_S_CTRL` for sensor controls, `VIDIOC_EXPBUF` for dmabuf export.
  Also opens the focuser subdev (`/dev/v4l-subdev*`) when found.

- **`IspPipeline`** (`IspPipeline.h`) — abstract base for demosaic / color
  processing. See [isp-pipeline.md](isp-pipeline.md) for the four backends.

- **`ImageConverter`** (`ImageConverter.cpp`) — libyuv-backed YUV/UYVY/YUYV →
  RGBA / JPEG paths, used when the sensor emits packed YUV (no ISP needed).

- **`Workers`** (`Workers.cpp/h`) — a generic thread pool. Currently used
  only by `CpuIspPipeline` / `ImageConverter` for parallelising row-wise
  work. **Not** used for request pipelining.

## Request lifecycle

All Camera3 per-frame work happens in `Camera::processCaptureRequest()`
(`Camera.cpp:741`). The flow is **strictly synchronous**, single-threaded,
single-buffer:

```
1. Parse request settings (exposure, gain, AF trigger, zoom, JPEG quality)
2. Apply settings via VIDIOC_S_CTRL (exposure, gain, frame length)
3. Handle AF trigger state machine (start/cancel sweep)
4. notifyShutter(frame_number, timestamp)
5. mDev->readLock()                     ← DQBUF one V4L2 buffer (blocking)
6. for each output buffer:
   a. Wait acquire fence
   b. Lock gralloc buffer
   c. Convert / process into gralloc buffer
      - RGBA: mIsp->processToGralloc() or processFromDmabuf()
      - BLOB: ImageConverter or mIsp->processSync() + JPEG encode
   d. Unlock gralloc buffer
7. AF sweep: evaluate sharpness, move VCM, update state
8. mDev->unlock(frame)                  ← QBUF buffer back to V4L2
9. Build result metadata
10. callbacks.process_capture_result(result)
```

The entire sequence runs under `mMutex` (held from step 1 through step 10).
Framework may have queued N capture requests ahead, but they are dispatched
one at a time.

## Stream configuration

`Camera::configureStreams()` receives the set of streams the framework wants
(preview, still, video, JPEG thumbnail, …). The HAL:

1. Picks a single V4L2 resolution to request from the sensor — typically
   the largest stream's resolution, clamped to supported sensor modes.
2. Calls `V4l2Device::setResolution()` → `VIDIOC_S_FMT` + `REQBUFS` +
   mmap + `VIDIOC_QBUF` all buffers.
3. Remaps `HAL_PIXEL_FORMAT_IMPLEMENTATION_DEFINED` → `RGBA_8888`.
4. Rejects ZSL and input streams.
5. Sets `max_buffers = 4` on every output stream (matches
   `V4L2DEVICE_BUF_COUNT`).

No per-stream buffer allocation happens in the HAL — gralloc buffers come
from the framework. The HAL owns only the V4L2 mmap buffers.

## Threading

- **Framework thread** — calls `processCaptureRequest`. This is where all
  real work happens.
- **Worker pool** (`Workers::gWorkers`) — CPU worker threads for parallel
  rows in `CpuIspPipeline` and `ImageConverter`. Synchronous fork/join:
  framework thread blocks until workers are done.
- **Camera3 callback thread** — the framework's thread on which we
  invoke `notify` and `process_capture_result`. Currently we call these
  from the framework thread itself (tail of `processCaptureRequest`),
  so there is no separate callback thread in practice.

There is **no dedicated capture / ISP thread**. This is the root cause of
the pipeline-depth-1 behaviour discussed in
[latency-and-buffers.md](latency-and-buffers.md).

## Static characteristics

Built once per camera in `Camera::getStaticCharacteristics()`
(`Camera.cpp:145`). The keys populated today are enumerated in
[camera3-compliance.md](camera3-compliance.md) along with the gaps. Values
are a mix of sensor-derived (resolutions, sensor area from `SensorConfig`),
hardcoded (optical properties — 3.3 mm focal length, 1.8 mm physical
sensor width), and inferred (VCM range from the focuser subdev via
`VIDIOC_QUERYCTRL`).

## AF state machine

Live in `Camera.cpp` across `mAfSweep*`, `mAfSettleFrames`, `mFocusPosition`.
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

Runtime knobs via setprop (`Camera.cpp` parses `ro.hal.camera.*`):

- `soft_isp` — `0`/`1` toggle between hardware ISP (Tegra `libnvisp_v3`)
  and software pipeline.
