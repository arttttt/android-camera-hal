# Preview latency and V4L2 buffers

What was solved during Tier 2 / Tier 3, what the current latency
model looks like, and where the remaining latency comes from on
the production async pipeline.

## Original problem (pre-Tier-2)

The HAL was effectively pipeline-depth-1: `processCaptureRequest`
ran synchronously under one mutex from
`mIsp->waitForPreviousFrame()` through
`mDev->readLock()` (DQBUF), GPU submit, and the
`process_capture_result` callback. Steady state with `BUF_COUNT=8`
and a ~40 ms processing budget would leave the kernel done-queue
filled with ~7 stale frames; `DQBUF` returns the **oldest**, so the
preview surface saw a frame ~7 frame_periods behind the actual
sensor output.

## Solutions landed

1. **Drain-to-latest in `V4l2Source` (Tier 2 PR `bac0ea0`).** First
   `DQBUF` of a request blocks (`poll(2)`-gated); subsequent ones
   are non-blocking and each successful drain `QBUF`s the previous
   slot back. Worst-case staleness drops from `BUF_COUNT × frame_period`
   to one frame regardless of pipeline jitter.

2. **Async capture thread (Tier 3 PR 3).** `BayerSource` /
   `V4l2Source` lifted the V4L2 dequeue loop onto its own thread;
   `CaptureThread` `poll(2)`s the V4L2 fd, drains, signals the
   `RequestThread` with `bayerReady(slot)`. The framework thread
   no longer waits on the sensor.

3. **Fence-fd polled GPU submits (Tier 3 PR 4).** The pre-Tier-3
   `waitForPreviousFrame()` was a synchronous `vkWaitForFences`
   call on the framework thread. Replaced with a depth-4 GPU-submit
   ring whose sync_fds (exported via `vkGetFenceFdKHR(SYNC_FD)`)
   are added to `PipelineThread`'s `poll()` set. `vkWaitForFences`
   now only fires on `flush()` / `closeDevice` / hang recovery —
   never on the hot path.

4. **`DelayedControls` (Tier 3 PR 6).** Per-control silicon delay
   (`SensorConfig::controlDelay[id]`, `2` for both EXPOSURE and
   GAIN on IMX179 / OV5693) is honoured by tagging each
   `S_EXT_CTRLS` write with the frame_number it lands on. Per-frame
   result metadata reports the value that *actually* applied at
   silicon, not the value the request asked for. Single source of
   truth for both auto (Ipa3A-driven) and manual
   (ApplySettings-driven) AE writes — neither side fights the
   other.

5. **Produce-once + JpegWorker / ResultThread split (Tier 3 PR 7).**
   Multi-output frames demosaic once (one `vkQueueSubmit` per
   frame, all outputs share the scratch ring slot) instead of N
   times. JPEG encode lives on `JpegWorker`; preview-only ctxs
   following a BLOB ctx keep flowing through `PipelineThread`
   while libjpeg works, and the FIFO gate on `ResultThread`
   preserves Camera3's monotonic `frame_number` ordering at the
   callback boundary.

## Current latency model

The per-frame `PERF:` line comes from `ResultDispatchStage`:

```
PERF: wait=<tBayerDq - tShutter>ms
      post=<tResultSent - tBayerDq>ms
      total=<tResultSent - tShutter>ms
      f=<frameNumber>
```

`tShutter` is set in `ShutterNotifyStage` (around the same time we
call `notify(SHUTTER, t)`). `tBayerDq` is when `BayerSource::
acquireNextFrame()` returns a fresh slot. `tResultSent` is right
before `process_capture_result`.

- **`wait` (tShutter → tBayerDq)** — request-side stages plus the
  wait on the sensor. With drain-to-latest + async capture this is
  bounded by one frame_period, modulo CaptureThread scheduling
  jitter. Typical 1080p / 30 fps: 30-35 ms.
- **`post` (tBayerDq → tResultSent)** — `DemosaicBlitStage`
  (compute demosaic + per-output blits / encodes / copies + cmd
  buffer record + `vkQueueSubmit`) on RequestThread, then the
  fence-fd wait + `StatsProcessStage` + `ResultDispatchStage` on
  PipelineThread / ResultThread. The fence-fd wait dominates
  because it covers the GPU work the submit kicked off. Typical:
  10-18 ms steady, spikes to ~25 ms.
- **`total`** — sum.

These segments **do not** include sensor exposure / readout / VI
DMA (those happen before `tShutter`), nor the framework's
`processCaptureRequest` push (set before `tShutter`).

## Steady-state numbers (1080p single-stream preview, IMX179)

```
demosaic + blit  ≈ 4 ms   GPU avg
NEON stats       ≈ 4 ms   CPU avg (parallel to GPU, hidden)
PERF post        7-18 ms  steady, 20-25 ms spikes
PERF wait        30-35 ms (one frame_period)
PERF total       ~45 ms (~22 fps frame-to-result)
FPS              30 (sensor cadence; result FIFO holds at sensor rate)
```

GPU and NEON CPU run in parallel, so the overlap between them is
hidden inside the fence-fd wait segment of `post`. The remaining
"stuck at ~22 fps frame-to-result on a 30 fps sensor" budget is
the per-stage queue handoff + ResultThread serialisation, not
intrinsic processing time.

## V4L2 buffer ring sizing

`V4L2DEVICE_BUF_COUNT = 8` is the production setting. Lower
values save nvmap (the V4L2 input ring is the largest single
allocation in `cameraserver`), but make drain-to-latest brittle —
the kernel's queue depth isn't the only contributor to staleness,
and a too-shallow ring exposes the sensor's internal pipelining
when CaptureThread misses a frame_period.

The ring is allocated as `V4L2_MEMORY_DMABUF` slots backed by
the Vulkan input ring's `OPAQUE_FD` exports; the V4L2 ring and the
Vulkan input ring are the same memory.

## What the latency model deliberately doesn't cover

- **Display compositor lag** — gralloc release fence to actual
  pixel-on-screen is `SurfaceFlinger` territory.
- **Camera3 `process_capture_request` push** — happens on the
  binder thread before the request enters our queue; runs in <1 ms
  but isn't measured.
- **JPEG encode** — `JpegWorker` runs concurrent with the next
  frames; its 100-150 ms cost shows up as a sustained gap between
  shutter and final still-image callback, but the preview FIFO
  isn't blocked by it (only the ctx that owns the BLOB stalls in
  ResultThread until JpegWorker clears `jpegPending`).
- **Sensor exposure + readout + VI DMA** — pre-shutter, not
  observable from the HAL.

The pre-Tier-3 plan in this doc included a `DelayedControls`
section as a future fix; that's now landed (item 4 above) and
the description here folds into the production model.
