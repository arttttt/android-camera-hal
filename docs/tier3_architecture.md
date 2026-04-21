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

Six threads per camera. One event loop each. Blocking work on its own
thread; framework-facing code never waits on GPU, sensor, or libjpeg.

| Thread          | Owner                       | Blocks on                   | Responsibility                                                                 |
|-----------------|-----------------------------|-----------------------------|--------------------------------------------------------------------------------|
| Main (binder)   | framework                   | nothing                     | HAL3 ops entry; `processCaptureRequest` pushes and returns 0                   |
| RequestThread   | `Camera`                    | condvar on queue            | Pop next request, apply controls, call `Ipa::processStats`, push to capture   |
| CaptureThread   | `V4l2Source`                | `poll()` on V4L2 fd + eventfd | QBUF / DQBUF, drain-to-latest, emit `bayerReady(slot)`                       |
| PipelineThread  | `VulkanIspPipeline`         | `poll()` on fence fds + eventfd | Vulkan record + submit, fence fan-out, ZSL ring, produce-once scratch     |
| ResultThread    | `Camera`                    | condvar on completion queue | FIFO-ordered `process_capture_result`, `notify` SHUTTER / ERROR                |
| PostprocWorker  | `JpegEncoder` (per stream)  | compute                     | libjpeg encode (one active at a time, spawned on demand)                       |

Framework callback threading: only `ResultThread` calls
`camera3_callback_ops::{process_capture_result, notify}`. No other
thread touches framework callbacks. This is a Camera3 requirement
(monotonic result ordering) enforced by single-thread serialisation,
not by an explicit CallbackSequencer.

## Data flow

```
                    processCaptureRequest(req)
                              |
                              v                    push + eventfd.write()
              +--- RequestQueue (bounded, N=4) -----+
              |                                    
              v
        RequestThread
          prepareRequest():
            - DelayedControls.push(controls, seq)
            - ipa.processStats(prevSeq, statsBuf)  [synchronous, in-thread]
            - reserve V4L2 slot
          push bayerRequest{slot, seq} -> CaptureQueue
                              |                    eventfd.write()
                              v
                        CaptureThread
                          poll(V4L2 fd, eventfd):
                            on eventfd: QBUF for queued requests
                            on V4L2: DQBUF, drain-to-latest, 
                                     emit bayerReady{slot, seq, timestamp}
                                       -> PipelineQueue
                              |                    eventfd.write()
                              v
                        PipelineThread
                          poll(fence_fd[], eventfd):
                            on eventfd:
                              demosaic bayer -> scratch   [once per frame]
                              for each output in req:
                                record blit / encode / copy
                              vkQueueSubmit (one batch)
                              vkGetFenceFdKHR -> register in poll set
                              push scratch into ZSL ring
                            on fence signal:
                              for each gralloc output: releaseFence -> ResultThread
                              for each jpeg output: dispatch to PostprocWorker
                              for stats: map + push to ResultQueue
                              emit requestComplete{seq, buffers[]}
                                -> ResultQueue
                              |                    cv.notify()
                              v
                        ResultThread
                          drain completions FIFO:
                            process_capture_result(seq, ...)
                            notify(SHUTTER / ERROR)
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
PR 5  DemosaicBlitStage moves to PipelineThread.pipeline
PR 6  ResultDispatchStage moves to ResultThread.pipeline
PR 7  (new) StatsProcessStage lands on PipelineThread.pipeline
PR 8  DemosaicBlitStage splits into DemosaicStage + per-stream BlitStage
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

Port of libcamera's `DelayedControls` (~280 LOC). Ring buffer of 16
slots, indexed by sequence number. Each control has a per-control
delay (in frames) sourced from `SensorTuning` JSON:

```json
"sensor": {
  "controlDelay": {
    "exposure": 2,
    "gain": 2,
    "vblank": 2,
    "vcm": 1
  }
}
```

API:

```cpp
class DelayedControls {
public:
    void push(uint32_t sequence, const ControlList& controls);
    ControlList applyControls(uint32_t sequence);
};
```

`RequestThread::prepareRequest` calls `push(seq, controls)`.
`CaptureThread::onSof(seq)` (or, if Tegra has no SOE exposed, right
before `VIDIOC_QBUF(seq)`) calls `applyControls(seq)` and writes the
batch via `V4L2Device::setControls`. `priorityWrite` flag for VBLANK
preserves ordering required by the driver to accept EXPOSURE.

`DelayedControls::applyControls(seq)` returns the controls pushed at
`seq - delay[ctrl]` for each control id — i.e. "the values requested
two frames ago now take effect".

## IPA (3A)

In-process, synchronous. No IPC isolation — our threat model does not
include closed-source algorithm blobs, and we cannot afford the
serialisation overhead.

Stats path:

1. PipelineThread runs a compute shader producing a statistics buffer
   (histogram + 16×16 patch-mean grid + optional sharpness per-patch)
   into a host-mapped `VkBuffer`.
2. On the frame-fence signal, PipelineThread calls
   `ipa->processStats(seq, statsBuf)` **from PipelineThread**. Tiny AE
   / AF / AWB computations (target < 1 ms on Tegra K1 CPU).
3. `ControlUpdate` returned by the IPA is pushed into `DelayedControls`
   by the PipelineThread's own completion handler.
4. Next frame (`seq + delay`) the new controls are applied at QBUF
   time.

No "3A thread". The IPA runs on whichever thread already has the
stats buffer mapped; cross-thread hop would waste latency.
We can always move it to its own thread later if AF becomes heavy
— the `Ipa` interface doesn't change.

Sequence coupling between stats and controls is explicit: every
`ControlUpdate` is tagged `effectSequence = inputSequence + delay`.
`ResultThread` uses this to report truthful `ANDROID_CONTROL_AE_STATE`
in result metadata.

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

Eight PRs, each shippable and testable independently. Each one
isolates a single concern so that review stays focused and FPS
regressions / gains are cleanly attributable via `git bisect`.

**Status:** PR 1-3 shipped — see roadmap.md for the landing summary
and memory note `project_tier3_progress.md` for the lifecycle /
streaming invariants that future PRs must preserve. PR 4-8 pending.

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

### PR 4 — DelayedControls

- Port of libcamera `DelayedControls` (~280 LOC), standalone.
- `SensorTuning` gains `controlDelay` field (`exposure`, `gain`,
  `vblank`, `vcm`).
- `RequestThread::prepareRequest` → `push(seq, controls)`;
  `CaptureThread::onBeforeQBuf(seq)` → `applyControls(seq)` writes
  batched `V4L2_CID_*`.
- Priority-write flag for `VBLANK` before `EXPOSURE`.
- Measurable: FPS unchanged; result metadata reports the controls
  that actually applied, not the latest requested.

### PR 5 — PipelineThread + fence-fd

- `PipelineThread` owns Vulkan record / submit.
- `vkGetFenceFdKHR` after each submit; fence fds registered in
  `poll()`.
- `VulkanIspPipeline::waitForPreviousFrame` deleted.
- Two-queue request model: `waitingRequests` / `inFlightRequests`
  with `maxInFlight = 3`.
- Measurable: **1080p preview 20 → 28–30 fps**. This is the PR the
  FPS bisect should point at.

### PR 6 — ResultThread

- `ResultThread` drains `CompletionQueue` FIFO.
- Framework callbacks (`process_capture_result`, `notify`) move off
  `PipelineThread`.
- No other behaviour change.
- Measurable: FPS unchanged; `notify` SHUTTER latency bounded by
  result-queue depth, not by GPU / JPEG work.

### PR 7 — Ipa + GPU statistics

- `Ipa` interface; `StubIpa` wraps today's per-frame 3A.
- New compute shader emits histogram + patch-mean grid into a
  host-mapped `VkBuffer`.
- Stats flow: `PipelineThread` calls `ipa->processStats(seq, stats)`
  on fence signal; `ControlUpdate` → `DelayedControls::push` with
  `effectSequence = seq + delay`.
- AE / AWB / AF start using GPU stats instead of locking gralloc for
  `SW_READ`.
- Measurable: `ANDROID_CONTROL_{AE,AWB,AF}_STATE` transitions driven
  by real stats; AF sweep no longer blocks on gralloc lock.

### PR 8 — Produce-once + JpegWorker

- `IspPipeline` grows `beginFrame(bayer)` / `blitToGralloc` /
  `blitToYuv` / `blitToJpegCpu` / `endFrame`.
- `VulkanIspPipeline` demosaics once per frame into
  `scratchImage`; per-output ops only blit / encode / copy.
- `BufferProcessor` loop deduplicates — N-stream requests no longer
  mean N demosaics.
- `PostProcessor` interface + `JpegEncoder` implementation;
  `JpegWorker` spawned for JPEG work off `PipelineThread`.
- Measurable: **preview + video multi-stream 13 → 28 fps**; JPEG
  encode doesn't stall preview.

Each PR: commit → `git pull github master` on build server →
`mmm hardware/camera` → scp `.so` → mocha-remote deploy → smoke-test
preview + video + snapshot → gate next PR on no FPS/correctness
regression.

## Performance expectations

| 1080p scenario | Today | After PR 2 | After PR 5 | After PR 8 |
|----------------|-------|------------|------------|------------|
| `processCaptureRequest` latency | ~50 ms | < 1 ms | < 1 ms | < 1 ms |
| FPS — preview only | ~20 | ~20 | ~28–30 | ~28–30 |
| FPS — preview + video | ~13 | ~13 | ~22 | ~28–30 |

Estimates based on current PERF log breakdown (~50 ms total,
~30 ms GPU drain). PR 5 assumes GPU depth = 2 (sensor + GPU overlap).
PR 8 assumes multi-stream demosaic deduplication.

## See also

- `docs/roadmap.md` — Tier 3 entry points to this document.
- `docs/latency-and-buffers.md` — motivation for request queue.
- `docs/open-source-references.md` — libcamera / Intel IPU6 / AOSP
  3_4 patterns we draw from.
- `docs/open-questions.md` — BT.709 / BT.601, NV21 output, AWB CCT.
