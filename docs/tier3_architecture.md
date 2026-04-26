# Tier 3 — pipeline architecture

Single-source-of-truth for the asynchronous, event-driven pipeline
refactor. This is the plan we implement against; every Tier 3 PR lands
as a subset of it.

## Goal

Break the implicit "caller does everything" model in
`Camera::processCaptureRequest` into a set of single-responsibility
threads communicating through bounded queues and sync-fd events. Remove
the synchronous GPU drain from the framework-facing thread, lift GPU
depth from 1 to N, and lay a foundation that cleanly extends to ZSL,
reprocess, real 3A, and multi-stream produce-once without rewrite.

Target improvement: 1080p single-stream throughput
`~20 fps → ~28–30 fps` without hardware changes. Multi-stream (preview
+ video + still) throughput limited by sensor cadence, not by HAL
orchestration.

## Thread topology

Seven threads per camera (ResultThread is rolled into PipelineThread
for the single-stream preview that currently ships; see PR 5 notes).
One event loop each. Blocking work on its own thread; framework-facing
code never waits on GPU, sensor, or libjpeg.

| Thread          | Owner                       | Blocks on                   | Responsibility                                                                 |
|-----------------|-----------------------------|-----------------------------|--------------------------------------------------------------------------------|
| Main (binder)   | framework                   | nothing                     | HAL3 ops entry; `processCaptureRequest` pushes and returns 0                   |
| RequestThread   | `Camera`                    | condvar on queue            | Pop next request, apply controls, capture Bayer, hand off to StatsWorker + Pipeline |
| CaptureThread   | `V4l2Source`                | `poll()` on V4L2 fd + eventfd | QBUF / DQBUF, drain-to-latest, emit `bayerReady(slot)`                       |
| StatsWorker     | `Camera`                    | `poll()` on job eventfd + stopfd | NEON reduce of raw Bayer into IpaStats; progressive over phaseCount submits |
| PipelineThread  | `VulkanIspPipeline`         | `poll()` on fence fds + eventfd | Vulkan record + submit, fence fan-out, ZSL ring, produce-once scratch     |
| (ResultThread)  | `Camera`                    | condvar on completion queue | Reserved. Single-stream preview runs result dispatch on PipelineThread directly |
| PostprocWorker  | `JpegEncoder` (per stream)  | compute                     | libjpeg encode (one active at a time, spawned on demand)                       |

Framework callback threading: the thread that runs
`ResultDispatchStage` is the sole caller of
`camera3_callback_ops::{process_capture_result, notify}`. Today that
is `PipelineThread`; when PR 7's multi-stream JpegWorker split lands
the stage moves to a dedicated `ResultThread`. This is a Camera3
requirement (monotonic result ordering) enforced by single-thread
serialisation, not by an explicit CallbackSequencer.

## Data flow

```
                    processCaptureRequest(req)
                              |
                              v                    push + eventfd.write()
              +--- RequestQueue (bounded, N=4) -----+
              |                                    
              v
        RequestThread
          ApplySettingsStage:
            - DelayedControls.push(controls, seq)
            - v4l2.setControls(batch)
          ShutterNotifyStage:
            - tShutter = systemTime(); notify SHUTTER
          CaptureStage:
            - BayerSource.acquireNextFrame()  [blocks on V4l2Source condvar]
            - stash VBuffer* in ctx
          StatsDispatchStage:
            - isp.invalidateBayer(slot); bayer = isp.bayerHost(slot)
            - StatsWorker.submit({bayer, w, h, pixFmt, seq})
          push ctx -> PipelineQueue
             |                              eventfd.write()
             |
             +------ StatsWorker (parallel branch)
             |         poll(job eventfd, stop eventfd):
             |           on job:
             |             phase == 0: start cycle, captures currentJob,
             |                         zeroes Partial
             |             every wake: computeRange(top .. bottom band of
             |                         patches for this phase) on currentJob
             |             on final phase: finalize Partial -> IpaStats,
             |                         publish under mutex
             v
        PipelineThread
          poll(fence_fd[], pipeline eventfd, stop eventfd):
            on eventfd:
              DemosaicBlitStage:
                record demosaic + blit for each output, vkQueueSubmit,
                vkGetFenceFdKHR -> register in poll set
            on fence signal:
              reap fence fds, once ctx fully signalled:
                StatsProcessStage (not alwaysRun):
                  StatsWorker.peek(&stats, &srcSeq)
                  batch = Ipa.processStats(srcSeq, stats)
                  DelayedControls.push(seq + delay[id], batch[id])
                ResultDispatchStage (alwaysRun):
                  process_capture_result / notify
                  BayerSource.releaseFrame(ctx.bayerFrame)
              BayerSource.flushPendingReleases()
```

## Synchronisation model

Every non-trivial wait uses a **file descriptor** and lives in a
`poll()` set. No `vkWaitForFences` on hot paths, no `cv.wait_until` for
GPU completion.

### Primitives

- **eventfd** — wake a thread when a queue has new work. One per thread.
- **V4L2 fd** — `poll(POLLIN)` for DQBUF readiness.
- **Vulkan fence fd** — `VK_KHR_external_fence_fd` with `SYNC_FD_BIT`.
  `vkGetFenceFdKHR` after `vkQueueSubmit` returns a sync_fd. Register
  it in the PipelineThread `poll()` set; when signalled, completion
  runs without CPU blocking.
- **Android sync_fd** — the same fd type the framework speaks in
  `release_fence` / `acquire_fence`. Direct pass-through: import
  `acquire_fence` via `vkImportFenceFdKHR` before submit; export
  `release_fence` via `vkGetFenceFdKHR` after submit.

### When `vkWaitForFences` is allowed

Only on three rare paths, all off the hot frame loop:

1. `flush()` — Camera3 API, drain in-flight on shutdown. Timeout 1000 ms.
2. `close()` — before destroying `VkDevice`. Timeout 1000 ms, then
   forcefully log and continue.
3. GPU-hang recovery — timeout 250 ms on any fence; on expiry mark
   the request `ERROR_REQUEST`, cancel downstream, continue.

These are the only places a HAL-owned thread blocks on the GPU.

### Queue design

All inter-thread queues are backed by `std::deque` under `std::mutex`,
woken by **eventfd** (not condvar) so they compose into `poll()`
sets.

```cpp
template <typename T>
class EventQueue {
public:
    EventQueue(size_t capacity);
    bool push(T item);          // false when full, caller decides policy
    std::optional<T> pop();     // non-blocking
    int fd() const;             // register in poll() set
private:
    size_t capacity;
    std::deque<T> items;
    std::mutex mutex;
    UniqueFd eventFd;
};
```

The primitive is always bounded (a hard cap in the data structure
protects against runaway memory on bugs) but the **effective cap** per
queue comes from the role, not the primitive:

| Queue            | Cap  | Overflow behaviour                                             |
|------------------|------|----------------------------------------------------------------|
| RequestQueue     | 256  | `LOG_ALWAYS_FATAL` — means the framework exceeded its own contract |
| CaptureQueue     | 8    | `LOG_ALWAYS_FATAL` — HAL accounting bug                        |
| PipelineQueue    | 8    | `LOG_ALWAYS_FATAL` — HAL accounting bug                        |
| CompletionQueue  | 16   | `LOG_ALWAYS_FATAL` — HAL accounting bug                        |

**Why no `-EBUSY` from `processCaptureRequest`.** The Camera3 contract
(`hardware/camera3.h`) defines only `0` / `-EINVAL` / `-ENODEV` as
valid returns from `process_capture_request`. `-EBUSY` is
undefined — framework behaviour on it is unspecified.

**Why RequestQueue doesn't need a tight cap.** Backpressure on the
framework is already expressed through `camera3_stream_t::max_buffers`,
set to `PIPELINE_MAX_DEPTH` (= 4) in `configureStreams`. The framework
can never submit more in-flight requests than it has buffers for;
since it holds those buffers and waits for us to return them via
`process_capture_result`, our RequestQueue can never grow beyond
`max_buffers × streamCount`. The 256 cap exists only to catch
contract violations (our accounting bug or a misbehaving framework);
on hit, log-fatal is preferable to silent memory growth.

**Why we can't drop Request objects on overflow.** Each request owns
framework-provided stream buffer handles and fences. The contract
requires exactly one result per request (success or error). Silently
dropping would leak buffers and stall the framework's buffer queue.
For genuine drop semantics we would instead complete the request
with `CAMERA3_MSG_ERROR_REQUEST` via `notify` — but in the normal
path we never reach this corner.

**Drain-to-latest is V4L2-only.** The `drain-to-latest` behaviour
introduced in Tier 2 and moving to `CaptureThread` in PR 3 applies
to **Bayer V4L2 buffers**, not to Request objects. Stale preview is
worse than dropped preview, so the sensor drop-to-newest is
appropriate there. Request-level drop is not.

`PIPELINE_MAX_DEPTH` in static metadata is reported as **4**.

## Interfaces (extension points)

Every cross-thread boundary is an interface, not a concrete. Tier 3
does not invent hypothetical extensibility — these interfaces exist
because there are already more than one conceivable implementation
we have in mind (ZSL bayer source, null IPA vs real IPA, JPEG vs
future encoders, V4L2 subdev capture vs single-node).

```
BayerSource
  virtual int start() = 0;
  virtual int stop() = 0;
  virtual int configure(resolution, pixelFormat, bufferCount) = 0;
  Signal<BayerFrame> bayerReady;

IspPipeline
  virtual int beginFrame(BayerFrame) = 0;       // demosaic -> scratch
  virtual int blitToGralloc(OutputBuffer) = 0;  // sample scratch
  virtual int blitToYuv(OutputBuffer) = 0;
  virtual int blitToJpegCpu(OutputBuffer) = 0;
  virtual int endFrame(SyncFd releaseFence) = 0; // submit + export fence
  virtual StatsBuffer statsFor(uint32_t sequence) = 0;

Ipa
  virtual ControlUpdate processStats(uint32_t sequence,
                                     const StatsBuffer&) = 0;

PostProcessor
  virtual int process(const ScratchView&, OutputBuffer,
                      Signal<void>& done) = 0;
```

`Signal<T>` is our own minimal event type (~80 LOC): slot list +
thread-local delivery mode. Not libcamera-port; just the idea.

`V4l2Source` implements `BayerSource`. `VulkanIspPipeline`
implements `IspPipeline`. `StubIpa` / `BasicIpa` implement `Ipa`.
`JpegEncoder` wraps `PostProcessor`.

Naming follows the no-Hungarian rule. Members are `bayerReady`, not
`mBayerReady`; constants are `constexpr size_t requestQueueCapacity = 4`
or `REQUEST_QUEUE_CAPACITY`, not `kRequestQueueCapacity`. Abstract
class names carry no `I` prefix (`BayerSource`, not `IBayerSource`) —
that prefix encodes role, which is exactly what the no-Hungarian rule
forbids.

## Pipeline composition

The async event-driven pipeline is expressed as four primitives:

- **`PipelineContext`** — per-frame state object, the baton. Holds the
  `CaptureRequest` (deep copy of framework input), the V4L2 bayer slot,
  GPU fence fds, per-output release fences, accumulated result
  metadata, status, and timestamps. Created once when a request is
  accepted; travels through stages; destroyed after result delivery.

- **`PipelineStage`** — abstract: `process(PipelineContext&) → Status`.
  Narrow interface, one concrete class per unit of work (apply
  settings, notify shutter, dequeue bayer, demosaic+blit, dispatch
  result, etc.). Stages own their own helper state (e.g. an
  `ApplySettingsStage` holds the `ExposureControl` + `AutoFocusController`
  instances), not the context's — they only read/write context fields.

- **`Pipeline`** — ordered `vector<PipelineStage*>` executed on one
  thread against a `PipelineContext`. Each thread owns its own
  `Pipeline`. The thread's event loop drives the pipeline:
  ```
  while (!stopRequested()) {
      poll(inputQueueFd, stopFd, ...);
      auto context = inputQueue.pop();
      pipeline.run(*context);   // iterate stages in order
      downstreamQueue.push(std::move(context));
  }
  ```

- **`InFlightTracker`** — registry of `PipelineContext*`s currently in
  flight. Owned by `Camera`, shared across threads. Operations:
  `add`, `removeBySequence`, `drainAll` (for `flush`/`close`),
  `count` (for metrics / debug).

### Migration path

```
PR 2  RequestThread.pipeline = [
          ValidateStage, ApplySettingsStage, ShutterNotifyStage,
          CaptureStage, DemosaicBlitStage, ResultDispatchStage
      ]

PR 3  CaptureStage moves to CaptureThread.pipeline
PR 4  DemosaicBlitStage moves to PipelineThread.pipeline
PR 5  ResultDispatchStage moves to ResultThread.pipeline
PR 6  (new) StatsProcessStage lands on PipelineThread.pipeline
PR 7  DemosaicBlitStage splits into DemosaicStage + per-stream BlitStage
```

Each thread-split PR is a **stage transfer**, not a rewrite: the stage
class implementation is unchanged, only its owning pipeline changes.
Cross-thread hand-off becomes a matter of `queue.push(std::move(ctx))`
at the end of the outgoing pipeline and `queue.pop(ctx)` at the start
of the incoming one.

### What this is NOT

- Not a DAG engine. Stages are linear; fan-out to multiple outputs
  (produce-once / sample-many) lives inside a single stage that
  iterates `context.request.outputBuffers`, not in the pipeline
  topology.
- Not a stage scheduler. Threads are not abstract workers; each thread
  has a fixed shape (orchestrator / poll-based V4L2 / Vulkan submitter /
  callback dispatcher). Stages are what happens inside each thread's
  dispatch.
- Not a message passing framework. Inter-thread communication is
  direct: one `EventQueue<std::unique_ptr<PipelineContext>>` per
  consumer thread.

## DelayedControls

Port of libcamera's `DelayedControls`. Ring buffer of 16 slots indexed
by `request.frameNumber`. Each control id carries a per-control delay
in frames — "the values requested N frames ago now take effect on
today's kernel frame". API:

```cpp
class DelayedControls {
public:
    enum ControlId { EXPOSURE, GAIN, COUNT };
    struct Batch  { bool has[COUNT]; int32_t val[COUNT]; };
    struct Config { int delay[COUNT]; int32_t defaultValue[COUNT]; };

    explicit DelayedControls(const Config&);
    void  reset();
    void  push(uint32_t seq, const Batch&);
    Batch applyControls(uint32_t seq) const;
};
```

`controlDelay` lives in `SensorConfig` (silicon property — not
per-integrator, not runtime-tunable), **not** in the tuning JSON. IMX179
and OV5693 both default to 2 frames as a libcamera-convention starting
point; verify empirically when a real 3A loop drives it.

### Scheduling

This is **not shipping as its own PR.** It is folded into the 3A PR
alongside the `Ipa` interface and stats shader. Reasons:

- On a Tegra / UVC-like driver there is no SOF hook, so the write
  itself stays on RequestThread (in `ApplySettingsStage`). A standalone
  DelayedControls is bookkeeping only — it does not move writes closer
  to the sensor's latch edge the way libcamera's SOF-indexed flow does.
- Without a real 3A loop there is no consumer writing new exposure /
  gain per frame, and therefore no race for `DelayedControls` to
  mediate. Truthful `ANDROID_SENSOR_EXPOSURE_TIME` in result metadata
  is the only standalone user-visible effect, and no known Android 7
  client checks it frame-accurately.
- Drain-to-latest (PR 3) introduces a known semantic gap: the ring is
  indexed in request-sequence time, kernel frames advance in
  kernel-frame time, and the two diverge whenever capture drops stale
  frames. Without a SOF-indexed counter (not available here) the gap
  is unfixable. Shipping an asymmetric abstraction now and papering
  over it later is worse than deferring.

When it lands, the write path is:

```
ApplySettingsStage (RequestThread):
    batch = exposure->compute(request.settings)
    delayedControls.push(seq, batch)
    v4l2.setControls(batch)                       // VIDIOC_S_EXT_CTRLS
    ctx.applied* = delayedControls.applyControls(seq)

Ipa::processStats (from PipelineThread, PR 6):
    update = ipa->computeAeAwb(statsBuf)
    delayedControls.push(seq + delay, update)     // future effect

closeDevice:
    delayedControls.reset()                       // session boundary
```

## IPA (3A)

In-process, synchronous. No IPC isolation — our threat model does not
include closed-source algorithm blobs, and we cannot afford the
serialisation overhead.

Stats producer lives on its own thread (`StatsWorker`) so CPU compute
overlaps the Vulkan demosaic + blit submit instead of serialising with
it. The original plan had the producer as a GPU compute shader; that
shipped transiently but spent GPU budget on stats rather than demosaic
and forced a temporal-subsample throttle to fit the frame budget. The
NEON path frees GPU for the picture path and lets stats run every
frame at ~4 ms peak CPU per frame (see Stats section below).

Stats path:

1. `RequestThread`'s `StatsDispatchStage` pulls the raw Bayer pointer
   for the ctx's V4L2 slot via `IspPipeline::bayerHost` and posts it
   to `StatsWorker` with the frame sequence.
2. `StatsWorker` runs a NEON-vectorised reduce (`NeonStatsEncoder`)
   over the raw Bayer, progressive across `phaseCount` submits, and
   publishes the finished `IpaStats` into a mutex-guarded slot.
3. On the frame-fence signal, `PipelineThread`'s `StatsProcessStage`
   calls `StatsWorker::peek(&stats, &srcSeq)` — returning the latest
   published snapshot — and invokes
   `ipa->processStats(srcSeq, stats)`. Target < 1 ms on Tegra K1 CPU.
4. `ControlUpdate` returned by the IPA is pushed into `DelayedControls`
   at `effectSequence = ctx.sequence + controlDelay[id]` so the next
   `ApplySettingsStage` lands it on the right kernel frame.

Sequence coupling between stats and controls is explicit: `IPA` sees
the sequence of the Bayer the stats were computed from; the push uses
the current-in-flight ctx sequence for the effect side. A one-frame
lag between produce and consume is expected in steady state — the
IPA treats stats as an input, not a blocker.

Raw-Bayer domain: because stats are computed on the uncorrected sensor
buffer before WB / CCM / gamma, `rgbMean` / `lumaHist` semantics match
libcamera's IPU3 / rkisp1 convention. `BasicIpa`'s AE loop consumes
this space directly (see below); AWB / AF will do the same.

### BasicIpa AE loop (landed)

`BasicIpa::processStats` is a P-controller with EMA damping:

1. Mean bin of the green-channel histogram over **all** 128 bins
   (saturated + black-clip included — skipping them was tempting but
   made AE chase overexposed results on high-contrast scenes; including
   them lets saturation pull the metric toward 1.0 and the loop back
   off). Divide by `(HIST_BINS - 1)` → mean luma in [0, 1]. Floor at
   `kAeMeasuredFloor = 0.02` so a pitch-black frame doesn't produce
   an unbounded ratio.
2. `ratio = kAeSetpoint / meanLuma`, clamp ∈ `[0.5, 2.0]` so a single
   very-bright / very-dark frame can't slam the sensor into saturation
   or black-clip in one go. Damp: `adjusted = 1 + (ratio - 1) × 0.3`.
3. Compute "last total at unity gain":
   `lastTotal = lastExposureUs × lastGain / gainUnit`. Multiply by
   `adjusted`. Clamp into `[kMinTotalUs, kMaxExposureUs × gainMax /
   gainUnit]`.
4. `SensorConfig::splitExposureGain` divides the new total back into
   `(exposureUs, extraGainQ8)`. It prefers to spend exposure first up
   to `kMaxExposureUs = 200000` (~200 ms, inside the default
   `frame_length`) before pushing into analog gain — dark-scene
   policy is **FPS priority**: we clamp rather than stretch
   `frame_length` to brighten.
5. Emit a two-entry `DelayedControls::Batch` with
   `EXPOSURE = newExposureUs` and `GAIN = newGain`, stash both as
   the new `last*`.

Tuning constants (`kAeSetpoint = 0.35`, `kAeDamping = 0.3`, ratio
clamps, `kAeMeasuredFloor`, exposure envelope) live in
`hal/ipa/BasicIpa.cpp`'s anonymous namespace. Moving them into
`SensorTuning` is a future enhancement when two sensors disagree; for
now IMX179 and OV5693 use the same values.

### ApplySettingsStage AE-mode branch

`ApplySettingsStage` is the single V4L2 writer for exposure + gain
and branches on `ANDROID_CONTROL_AE_MODE`:

- **`AE_MODE_OFF` — manual.** Parse request metadata via
  `ExposureControl::onSettings`, which clamps into sensor limits and
  calls `V4l2Device::setControl`. Then publish the applied values
  into `DelayedControls::push(request.frameNumber, …)` so
  `ResultMetadataBuilder::applyControls(frame + delay)` returns the
  same numbers we just wrote. `StatsProcessStage` detects
  `AE_MODE_OFF` and **skips** its own `DelayedControls::push` — the
  IPA's compute still runs (useful for a clean switch back to
  auto), but the result is kept out of the ring so the framework's
  manual authority is never fought.

- **`AE_MODE != OFF` — auto.** Read
  `DelayedControls::pendingWrite(request.frameNumber)` — a new
  companion to `applyControls(seq)` that returns
  `slot[seq]` directly ("what to write at frame seq", the auto
  branch's consumer API) rather than
  `slot[seq − delay[id]]` ("what was in physical effect at frame
  seq", the result-metadata consumer API). If either
  `EXPOSURE` / `GAIN` is set, `ExposureControl::applyBatch` writes
  them in one `VIDIOC_S_EXT_CTRLS` call. If the ring has nothing
  for this slot (cold start, or `StubIpa`-era empty batches), we
  fall back to the manual `onSettings` path so the sensor never
  freezes during bring-up.

`DelayedControls` is therefore the single source of truth for
applied exposure + gain. Producers: ApplySettings (manual branch,
`slot = frameNumber`) and `StatsProcessStage` (IPA push,
`slot = sequence + delay[id]`). Consumers: `pendingWrite` from
ApplySettings' auto branch on RequestThread, `applyControls` from
`ResultMetadataBuilder` on PipelineThread. The class owns a mutex
since those four call sites span three threads.

### AWB / AF inside BasicIpa (pending)

AWB will be gray-world over `rgbMean[16][16][3]` (still pre-WB /
pre-CCM), emitting WB gain batches into `DelayedControls` via the
same IPA push path. AF will feed `sharpness[16][16]` into
`AutoFocusController` in place of its current gralloc
`SW_READ_OFTEN` sharpness read — removing the last CPU lock on the
output surface. Both extend `BasicIpa::processStats`; no new
threading.

## Stats — NEON on StatsWorker

`NeonStatsEncoder` owns the ARMv7-NEON reduce from a raw Bayer slot
into the IpaStats layout (128-bin green-channel histogram, 16×16
patch-mean RGB, 16×16 Tenengrad sharpness). The producer thread
(`StatsWorker`) carries state across submits so one IpaStats cycle
spans `phaseCount` incoming frames:

- Each submit advances one phase. Phase 0 of a cycle captures the
  Bayer pointer and zeroes a `Partial` accumulator; later phases run
  `computeRange(pyStart, pyEnd)` over an equal-sized band of patch
  rows against the captured Bayer; the final phase calls `finalize`
  to convert the accumulator into an `IpaStats` and publishes under
  the output mutex.
- `phaseCount` is a single compile-time constant in `StatsWorker`.
  Default 2 → per-frame peak CPU halved (~4 ms on 720p) vs a
  one-shot compute, and the design budget for one stats result is
  `frame_period × phaseCount` (22 ms at 60 fps). `phaseCount = 1`
  degenerates to full-image compute on every submit; higher values
  trade stats refresh rate for lower peak CPU and room for richer
  future stats.

Bayer lifetime across the multi-frame cycle is safe on the standing
V4L2 ring: `V4L2DEVICE_BUF_COUNT = 8` slots, HAL holds at most three
in flight, so a released slot rotates back through V4L2's fill queue
over five to seven sensor periods (~80–120 ms at 60 fps). One cycle
spans `phaseCount × frame_period` (~22 ms), well inside that window —
no memcpy or explicit slot-pinning API is needed.

Cache coherency: `IspPipeline::invalidateBayer(slot)` is called at
dispatch time (RequestThread) before the pointer is handed to the
worker, so the CPU sees the DMA-written content V4L2 produced. The
Vulkan backend's `VulkanInputRing` allocates slots as
`HOST_VISIBLE | HOST_CACHED` when the driver supports it; the
`InvalidateMappedMemoryRanges` call is the standard flush-from-GPU
recipe.

## Produce-once / sample-many (Tier 3.5, same architecture)

PipelineThread demosaics **once** per frame into `scratchImage`. Each
output buffer in the request performs a blit / encode / copy that
samples `scratchImage` via `sampler2D` (RGBA output) or compute SSBO
write (NV12 output). All recorded in one command buffer, submitted
once.

Multi-stream (preview + video + jpeg) cost:

| Stage              | Current (per-stream demosaic)  | Produce-once                    |
|--------------------|--------------------------------|---------------------------------|
| Demosaic           | N× per frame                   | 1× per frame                    |
| Blit               | N× per frame                   | N× per frame                    |
| Total GPU work     | O(N × scratchCost + N × blit)  | O(scratchCost + N × blit)       |

AF sharpness metric samples `scratchImage` directly instead of
locking the gralloc output for `SW_READ` — one more CPU fallback
removed.

## ZSL / reprocess (slot in Request, impl Tier 4)

`Request` carries an optional `inputBuffer` from day one:

```cpp
struct Request {
    uint32_t sequence;
    ControlList controls;
    std::optional<InputBuffer> inputBuffer;  // ZSL / reprocess
    std::vector<OutputBuffer> outputBuffers;
};
```

When `inputBuffer.has_value()`, PipelineThread bypasses
V4L2 / demosaic and sources from:

- **ZSL path** — ring buffer of recent `scratchImage` snapshots
  (N=4–6) kept in PipelineThread. `RequestThread` picks the closest
  timestamp.
- **Reprocess path** — framework-provided YUV/BLOB input, imported
  as a Vulkan external image.

The ring buffer implementation is Tier 4; the Request shape is Tier 3.
Not having `inputBuffer` in the Request struct from the start means
ripping through every thread's queue type later.

## Lifecycle

Start-up order (from `configure_streams` → first frame):

1. `Camera::configureStreams` — Blocking: validate, allocate gralloc
   buffers, configure IspPipeline.
2. `start()` chain: V4L2 STREAMON → CaptureThread loop →
   PipelineThread loop → RequestThread loop → ResultThread loop.
   Each thread created via `ThreadBase::start()`, signals ready via
   a startup barrier.

Shutdown order (reverse, explicit):

1. `Camera::flush()` — stop accepting new requests, drain
   `RequestQueue` with `ERROR_REQUEST` for pending, `vkWaitForFences`
   for in-flight GPU work.
2. `close()` — `stop()` each thread via its stop eventfd, then
   `join()` in reverse of start order. RequestThread joins first
   (no more upstream), ResultThread joins last (drain completions).
3. `~Camera()` — destroy `VkDevice`, close V4L2.

Every thread is a subclass of:

```cpp
class ThreadBase {
public:
    void start();
    void stop();
    bool isRunning() const;
protected:
    virtual void threadLoop() = 0;
    int stopFd() const;
};
```

Exit discipline: threads leave `threadLoop()` only on stop eventfd
signal. No "check `running` flag at top of loop iteration" races.

## Error handling

- **V4L2 timeout / ESTALE on DQBUF**: CaptureThread marks the slot's
  request `ERROR_BUFFER`, pushes to CompletionQueue, continues.
- **Vulkan submit error**: PipelineThread marks request
  `ERROR_REQUEST`, triggers `flush()` on fatal classes
  (VK_ERROR_DEVICE_LOST).
- **IPA processStats error**: logged, ignored for that frame. 3A
  stays at the previous known-good `ControlUpdate`.
- **Framework callback error (unlikely, framework contract)**: logged,
  continue.

Every thread wraps `threadLoop()` in a top-level try/catch that logs
and signals camera-level error to the framework (`notify(ERROR_DEVICE)`).
No silent death.

## Non-goals for Tier 3

- **IPC-isolated IPA** (libcamera IPAProxy). No closed-source blobs to
  protect.
- **Pipeline-handler registry / plugins**. Exactly one pipeline —
  `VulkanIspPipeline`. YAGNI.
- **Timeline semaphores** (Vulkan 1.2). Tegra K1 is Vulkan 1.0.
- **Multiple Vulkan queues** for parallel dispatch. One graphics /
  compute queue (Tegra K1 exposes one). Tier 4 if a second queue
  becomes useful for 3A stats compute in parallel with ISP.
- **CallbackSequencer** (Exynos). Single `ResultThread` gives FIFO by
  construction.
- **Full port of libcamera `Object`/`Signal`/`EventDispatcher`**. We
  use the design pattern, write ~300 LOC of our own.
- **ZSL ring buffer impl**. Slot exists in `Request`; ring buffer is
  Tier 4.
- **Reprocess impl**. Slot exists; framework integration is Tier 4.

## Implementation sequencing

Originally eight PRs. The `DelayedControls` PR folded into the 3A PR
once it became clear the former has no real consumer without the
latter (see the DelayedControls section above for the full rationale);
the NEON-stats rewrite and the BasicIpa AE + ApplySettings branch were
sized as follow-ups after `StatsProcessStage` + `StubIpa` had landed,
so what started as seven landings became nine. Each one isolates a
single concern so that review stays focused and FPS regressions /
gains are cleanly attributable via `git bisect`.

**Status:** all PRs shipped — see roadmap.md for the landing summary
and memory note `project_tier3_progress.md` for the lifecycle /
streaming invariants that future work must preserve.

### PR 1 — Threading primitives (infra only) — SHIPPED

- `base/ThreadBase`, `base/EventQueue<T>`, `base/Signal<T>`,
  `base/UniqueFd` (eventfd wrapper).
- Unit tests exercising queue / signal / eventfd wake behaviour.
- No consumers yet.
- Measurable: compiles; unit tests pass.

### PR 2 — RequestQueue + RequestThread — SHIPPED

- `RequestQueue` concrete specialisation.
- `RequestThread` wraps the current synchronous flow (still calls
  today's V4L2 + Vulkan sync path).
- `Camera::processCaptureRequest` pushes and returns.
- Measurable: `processCaptureRequest` returns in < 1 ms; downstream
  timing unchanged.

### PR 3 — CaptureThread split — SHIPPED

- `BayerSource` interface; `V4l2Source` implementation owns
  `CaptureThread`.
- `poll()` on V4L2 fd; `Signal<BayerFrame> bayerReady`.
- Drain-to-latest logic migrates out of `V4l2Device::readLock` into
  `CaptureThread::onBufferAvailable`.
- `RequestThread` pushes `bayerRequest`, awaits `bayerReady`.
- Measurable: DQBUF off RequestThread; FPS unchanged.

### PR 4 — PipelineThread + fence-fd — SHIPPED

- `PipelineThread` owns Vulkan record / submit.
- `vkGetFenceFdKHR` after each submit; fence fds registered in
  `poll()`.
- `VulkanIspPipeline::waitForPreviousFrame` dropped from the hot path.
- Slot ring of depth 4 with fence-fd export; in-flight cap = 1 ctx
  in PipelineThread (raised from 1 once CPU stats came off the hot
  path in PR 6.5).
- Demosaic shader perf pass landed on the same tier: float-recip
  `readPixel`, cooperative shared Bayer tile, 16×16 workgroup,
  sRGB LUT, Blit uv precompute, NV12 stride. 1080p demosaic
  47 ms → 3.4 ms.
- Measurable: PERF `wait` segment (GPU-drain dominated) collapsed
  from ~55 ms to near zero.

### PR 5 — ResultThread split — ROLLED INTO PR 4

A dedicated `ResultThread` was unnecessary for single-stream preview:
result dispatch already ran off the binder thread via PipelineThread,
and the callback-ordering guarantee is preserved by the single stage
running on a single thread. The split lands with the multi-stream
JpegWorker in PR 7, where a second consumer (JPEG encode) would
otherwise force interleaving of the result queue across threads.

### PR 6 — Ipa + DelayedControls + stats plumbing — SHIPPED

- `Ipa` interface under `hal/ipa/`; `StubIpa` returns empty batches
  so `ApplySettingsStage`'s direct push remains authoritative until
  `BasicIpa` lands.
- `DelayedControls` class (`isp/sensor/DelayedControls.{h,cpp}`);
  `controlDelay` lives in `SensorConfig` (silicon property, not JSON).
- `StatsProcessStage` inserted between fence reap and
  `ResultDispatch` on `PipelineThread`; skipped on errored ctxs.
- Camera's `closeDevice` resets `Ipa` and `DelayedControls` alongside
  the other session-boundary cleanups.

Stats producer originally shipped as a GPU compute shader
(`VulkanStatsEncoder`) writing into a host-mapped `VkBuffer`, with
temporal subsample every 2nd frame. That path was replaced post-
PR 6 by the NEON stats worker described in the Stats section above
— see PR 7 (renumbered from the original sequence) for what remains
open.

### PR 6.5 — NEON stats on StatsWorker — SHIPPED

- `NeonStatsEncoder` under `hal/ipa/` — stateless raw-Bayer reducer
  with both all-at-once `compute()` and progressive
  `computeRange(pyStart, pyEnd) + finalize()` entry points.
- `StatsWorker` (`hal/ipa/`) — `ThreadBase` subclass that runs the
  progressive compute across `phaseCount` submits, exposing
  latest-wins `submit()` and snapshot `peek()`.
- `StatsDispatchStage` on RequestThread (`hal/pipeline/stages/`) —
  pulls `bayerHost(slot)` + `invalidateBayer(slot)` from the ISP
  backend and posts a Job to StatsWorker on every frame.
- `IspPipeline::bayerHost` / `invalidateBayer` virtuals added to the
  base and overridden in `VulkanIspPipeline` to forward to the
  existing `VulkanInputRing` CPU mapping.
- Vulkan GPU stats backend (`VulkanStatsEncoder`, `StatsCompute.h`,
  `IspPipeline::mappedStats` / `invalidateStats`) removed once the
  CPU path was validated on-device; binary shrank ~8 KB.

### PR 6.6 — BasicIpa AE + ApplySettings AE-mode branch — SHIPPED

- `hal/ipa/BasicIpa` — P-controller with EMA damping over the
  raw-Bayer green-channel mean-luma histogram (see IPA section for
  the math). Replaces `StubIpa` in `Camera::buildInfrastructure`.
- `hal/pipeline/stages/ApplySettingsStage` — branches on
  `ANDROID_CONTROL_AE_MODE`. Manual (`OFF`) parses request settings
  and writes V4L2 directly, then publishes into `DelayedControls`
  so result metadata is consistent with what was written. Auto
  (non-`OFF`) reads `DelayedControls::pendingWrite(frameNumber)`
  and pushes one `VIDIOC_S_EXT_CTRLS` via `applyBatch`, with a
  cold-start / `StubIpa` fallback to the manual path.
- `isp/sensor/DelayedControls` — added `pendingWrite(seq)` as the
  auto-branch consumer API (returns `slot[seq]` directly, distinct
  from `applyControls(seq)` which returns `slot[seq − delay[id]]`
  for result metadata). Added a mutex: producers span PipelineThread
  (StatsProcessStage) + binder (ApplySettings); consumers span
  RequestThread (pendingWrite) + PipelineThread
  (ResultMetadataBuilder). The class is now thread-safe by design.
- `hal/3a/ExposureControl` — added `applyBatch(const Batch&)` for
  the auto branch; fixed a bug where `onSettings` stored
  `mAppliedExposureUs` in lines rather than µs.
- `hal/pipeline/stages/StatsProcessStage` — when `AE_MODE == OFF`,
  skips the `DelayedControls::push` so the framework's manual
  authority isn't fought by the IPA (compute still runs, for clean
  switch-back to auto).
- `hal/Camera::buildInfrastructure` — `DelayedControls` init moved
  ahead of `ApplySettingsStage` in the wiring order since
  ApplySettings now depends on it in both branches.

### PR 7 — Produce-once + JpegWorker + ResultThread split — SHIPPED

- `IspPipeline` grew `beginFrame` / `blitToGralloc` / `blitToYuv` /
  `blitToJpegCpu` / `endFrame`. `VulkanIspPipeline` records demosaic
  once into `mScratchImg` per frame and each output blits / encodes /
  copies from there inside a single `vkQueueSubmit`. Framework
  acquire_fence sync_fds get imported as binary `VkSemaphore`s via
  `VK_KHR_external_semaphore_fd` (probed + enabled at instance + device
  level) so the recording thread never blocks on framework backpressure.
  Per-output `release_fence`s come from `vkQueueSignalReleaseImageANDROID`,
  one call per gralloc image, sharing the post-submit queue ordering.
- `BufferProcessor::processOne` switched to the blit-only API; YUV path
  records the GPU NV12 encode at record time and defers the libyuv
  repack into the gralloc to `finalizeCpuOutputs` on PipelineThread
  after fence reap. BLOB path records `vkCmdCopyImageToBuffer(scratch →
  jpeg ring slot)` and stashes a `JpegSnapshot` (handle into a
  SLOT_COUNT-deep host-mapped RGBA8 ring) in the ctx.
- New `PostProcessor` abstract under `hal/pipeline/`. `JpegEncoder`
  implements it (libjpeg + EXIF Orientation marker); the IspPipeline
  dependency dropped along the way since `JpegSnapshot` is the only
  producer-side input now.
- Dispatch side splits in two:
  - `ResultThread` (`ThreadBase`) owns `ResultDispatchStage`,
    `BayerSource::flushPendingReleases`, and
    `InFlightTracker::removeBySequence`. Single consumer of
    `mResultQueue`. Maintains an internal pending FIFO so it can peek
    head readiness without popping; only dispatches the ready prefix
    (errored OR `jpegPending == 0`).
  - `JpegWorker` (`ThreadBase`) pulls per-output `JpegWorker::Job` from
    `mJpegQueue`, calls `BufferProcessor::finalizeBlobOutput`, and on
    the last-decrement of `ctx->jpegPending` notifies a shared
    `EventFd` that `ResultThread` polls.
- `PipelineContext` grew a `std::atomic<int> jpegPending` set by
  PipelineThread before queueing JPEG jobs. Camera3 monotonic-ordering
  invariant preserved by ResultThread's serial single-thread design.
- Lifecycle: ResultThread → JpegWorker → PipelineThread → RequestThread
  (start order); reverse for stop, with JpegWorker draining its queue
  between PipelineThread and ResultThread so any ctxs parked on
  `jpegPending` get released before dispatch shuts down.
- Legacy `processToGralloc` / `processToYuv420` / `processToCpu` and
  their helpers (`recordGrallocBlit`, `submitWithReleaseFence`,
  `recordDemosaicAndYuvEncode`, `recordAndSubmit`, `mOutBuf`) deleted
  once no consumer remained.
- Measurable on device: BLOB encode (~120 ms at 1080p) runs on
  JpegWorker; PipelineThread keeps admitting ctxs through the
  encode window so sensor frames are not dropped (verified via
  f=N+1 `wait=0ms` immediately after the BLOB frame). ResultThread's
  FIFO gate means subsequent preview dispatches still wait for the
  BLOB frame's encode — that's the inherent Camera3 ordering cost.

Each PR: commit → `git pull github master` on build server →
`mmm hardware/camera` → scp `.so` → mocha-remote deploy → smoke-test
preview + video + snapshot → gate next PR on no FPS/correctness
regression.

## Performance expectations

| Scenario | Pre-Tier-3 | Post-PR-2 | Post-PR-4 | Post-PR-6.5 | After PR 7 |
|----------|------------|-----------|-----------|-------------|------------|
| `processCaptureRequest` latency | ~50 ms | < 1 ms | < 1 ms | < 1 ms | < 1 ms |
| 720p preview FPS | 61 (vsync) | 61 (vsync) | 61 (vsync) | ~88 (no cap) | ~88 (no cap) |
| 1080p preview FPS | ~20 | ~20 | ~28–30 | ~28–30 | ~28–30 |
| 1080p preview + video FPS | ~13 | ~13 | ~22 | ~22 | ~28–30 |

PR 4 assumes GPU depth ≥ 2 (sensor + GPU overlap). PR 6.5 (NEON
stats) does not change fps vs PR 4 for the GPU-bound paths but
clears ~3 ms GPU time per frame that the original GPU-stats
dispatch consumed, leaving headroom for PR 7's multi-stream work.
PR 7 assumes multi-stream demosaic deduplication.

## See also

- `docs/roadmap.md` — Tier 3 entry points to this document.
- `docs/latency-and-buffers.md` — motivation for request queue.
- `docs/open-source-references.md` — libcamera / Intel IPU6 / AOSP
  3_4 patterns we draw from.
- `docs/open-questions.md` — BT.709 / BT.601, NV21 output, AWB CCT.
