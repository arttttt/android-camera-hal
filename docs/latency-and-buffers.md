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
PERF: sensor=<tShutter - tAccepted>ms
      permit=<tBayerDq - tShutter>ms
      post=<tResultSent - tBayerDq>ms
      total=<tResultSent - tAccepted>ms
      f=<frameNumber> sensorFrame=<sensor frame>
```

`tAccepted` is when `Session::submit` takes the request. `tShutter`
is the frame's start (the capture engine's shutter notify).
`tBayerDq` is when the capture engine hands the frame's Bayer slot
over. `tResultSent` is right before `process_capture_result`.

- **`sensor` (tAccepted → tShutter)** — how long the request waited
  for its sensor frame. `requestsInFlight` (6, `Session::Config`)
  bounds this: a request waits as many frame periods as the depth.
  This is the depth's price, and it is paid before anything is
  captured.
- **`permit` (tShutter → tBayerDq)** — the frame's start to its
  Bayer slot: the sensor's exposure, readout and VI DMA, plus the
  capture engine's turn. About one frame period.
- **`post` (tBayerDq → tResultSent)** — the frame's processing:
  `DemosaicBlitStage` (compute demosaic + per-output blits /
  encodes / copies + cmd buffer record + `vkQueueSubmit`), the
  fence-fd wait that covers the GPU work, the YUV finish and 3A on
  the finish engine, the JPEG hand-off, and the result.
- **`total`** — acceptance to result: what the framework sees.

The `permit`/`post` split is where the processing budget lives; the
`sensor` segment is depth, not work.

These segments **do not** include the framework's
`processCaptureRequest` push (set before `tAccepted`), nor the
sensor's exposure and readout other than as the `permit` wait.

## Steady-state numbers (1080p, 24.10.2026)

Measured on 24.10.2026, 1080p, bright scene, auto exposure, both
cameras, with `persist.camera.perf` on (the PERF line above,
`PERF-GPU`, and the stats engine's PERF lines). "Recording" is one
YUV stream: rear 30 fps, front 24 fps; **no sensor frame was lost**
on either.

Recording, median / p95:

```
                      rear 30 fps     front 24 fps
result interval       33 / 43 ms      41 / 62 ms
post                  25 / 53 ms      28 / 62 ms
NEON phase            6.5 / 12 ms     7 / 15.6 ms
IPA                   1.1 / 3.0 ms    1.2 / 3.9 ms
```

`PERF-GPU`, rear recording at 30 fps: demosaic 13-14 ms, outputs
5-6 ms, total 18-20 ms on average; the worst frame 31-37 ms. With
CPU/GPU/EMC pinned at 1836/612/924 MHz the demosaic is ~10.4 ms —
the governor's clock, not the kernel's work, is most of the
difference.

Preview runs at ~16.6 fps in a bright scene too, and that is a
policy, not a power limit: auto exposure spends the light on
exposure first — it stretches the frame to 15 fps, the bottom of the
request's [15, 30] fps range, before it raises gain — so the preview
cadence is exposure-bound. In preview the NEON phase and the GPU
are slower than in recording; the CPU/GPU governors are the
difference, not the kernels (same code, lower clocks while the
frame is long).

GPU and NEON run in parallel — the NEON kernel reads the frame's
Bayer slot while the GPU works the same frame — so their costs
overlap inside `post`. In recording the pipeline holds the sensor's
cadence (30 / 24 fps, no frames lost); the preview's 16.6 fps is
the exposure policy above, not the processing budget.

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
- **JPEG encode** — the JPEG engine (`core/engines/JpegEngine`) runs
  concurrent with the next frames' capture and processing; what it
  delays is their results. The result engine dispatches strictly in
  submission order and waits for each frame to settle (its
  `inPipeline` tokens), and the BLOB's frame settles only after its
  encode — so the results behind a photo wait ~100-150 ms, while
  their capture and GPU work go on.
- **Sensor exposure + readout + VI DMA** — inside `permit`, not
  separately observable from the HAL.

The pre-Tier-3 plan in this doc included a `DelayedControls`
section as a future fix; that's now landed (item 4 above) and
the description here folds into the production model.
