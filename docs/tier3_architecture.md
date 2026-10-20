# Execution model of the core

How `libmochacamera` runs a frame: its threads, what each one does,
who owns a frame and its resources on the way, and how the core stops,
flushes and fails. The module split and the Camera3 frontend are in
[architecture.md](architecture.md).

The file keeps its old name for the links into it; "Tier 3" was the
first threaded pipeline (RequestThread, CaptureThread, PipelineThread,
ResultThread, JpegWorker, StatsWorker, each with its own loop and
queues). The core rewrite replaced every one of them with engines on
one primitive; see [History](#history).

## The engine

`core/engine/Engine.h`. An engine is a `JobQueue` and the one thread
that works it -- and the only way the core has a thread. A component
owns its engine and gives it a handler; nothing inherits a thread.
Engines are built and started when the stream starts, stopped and
destroyed when it stops; whatever is worth keeping across that lives in
the component, not in the engine.

The handler answers each job with a **Step**:

- `done(result)` -- finished;
- `refused(why)` -- not done and not going to be;
- `wait(fd, onReady)` -- started; the engine keeps the job, polls `fd`
  on its own thread and calls `onReady` when it is readable, which
  again answers with a Step (a job may wait more than once).

There is no third kind of handler. Everything happens on the engine's
thread: the handler, continuations, the callbacks of watched fds
(`watch(fd, cb(revents))`), the stall callback, and the destruction of
a finished job -- so what a job holds (a frame token, a Bayer lease, a
permit) is let go there.

The thread sits in one `poll`: its wake (submit, cancel, stop), the fds
of waiting jobs, and the watched fds. The wake is drained *before* the
queue is read, so a job queued at any moment is either seen now or
wakes the next poll; the lost wake that hung the old pipeline cannot be
written again. A failed poll puts the engine into refusing: it leaves
its loop and every later submit is refused, rather than the thread
spinning.

`EngineOptions`:

- `maxRunning` -- jobs taken at once, running or waiting (1 where jobs
  must not overlap);
- `keepOutcomes` -- whether an owner collects outcomes (`outcomeFd`,
  `ackOutcomes`); the core's engines pass work on themselves and keep
  none;
- `stopGrace` -- how long `stop()` lets waiting jobs finish (a fence,
  say) before cancelling them.

`setStallWatch(limit, cb)` calls `cb` once for a job that has waited
longer than `limit` since its first wait; the job keeps waiting, and
what a stall means is the component's call -- in the core, a device
error: a sensor that stopped sending frames, a GPU that hung.

`JobQueue` (`core/engine/JobQueue.h`) is the composer's queue, copied:
states Queued / Running / Done / Refused / Dropped / Cancelled, drop
policies None / Bounded / LatestOnly, `cancel`, `tryResolve`. Its
`done()` notifies resolvers of a cancelled job too (the composer's copy
did not, and hung its waiter).

## The six engines

```
 submit ─┬─────────────────────────────────────────────────► Result ──► sink
         │                                                    ▲ (frame order,
         ▼                                                    │  tokens == 0)
      Capture ──(permit)──► Isp ──(fence)──► Finish ──► Jpeg ─┘
         │                   GPU                │
         └── BayerLease ──► Stats ──(peek)──────┘ 3A
```

| Engine | maxRunning | Waits on | Does |
|---|---|---|---|
| **Capture** (`CaptureEngine`) | 1 | permit, V4L2 node | permit for Isp → `ApplySettingsStage` + `ShutterNotifyStage` → newest Bayer → crop → lease to Stats → frame + permit to Isp. Owns the V4L2 queue (one writer). Stall watch 3 s → device error. |
| **Stats** (`StatsEngine`) | 1, LatestOnly | -- | NEON statistics in `kPhaseCount` = 2 phases over consecutive jobs; the last phase publishes for `peek()`. A newer job supersedes a waiting one and its lease. |
| **Isp** (`IspEngine`) | 1 (`kDepth`) | GPU fence | records the demosaic and every output's blit into one submit (`DemosaicBlitStage`), waits on the fence, gives the Bayer lease back, passes the frame to Finish; the permit goes with the job. Stall watch and stop grace 1 s. |
| **Finish** (`FinishEngine`) | 1 | -- | commits the frame, finishes YUV outputs on the CPU (`BufferProcessor`), runs 3A (`StatsProcessStage`: `Ipa3A`, partial results, AF), hands each BLOB output to Jpeg. |
| **Jpeg** (`JpegEngine`) | 1 | -- | one job per BLOB output; owns that output's JPEG snapshot (a slot of the ISP's ring); encodes with EXIF and thumbnail. |
| **Result** (`ResultEngine`) | 1 | the frame's tokens | takes frames in submission order, waits until no job holds a token on the frame, hands the result back (`ResultDispatchStage` → `ResultSink::frame`), lets the registry forget it. |

Settings and shutter run on the capture engine, not on one of their
own: `ApplySettingsStage` takes the IPA's decision filed for this very
frame number and the shutter is stamped as it runs, so both must go in
step with the capture -- a separate engine would race through the whole
queue at once.

The capture engine takes one job at a time: a second waiting job would
take a good frame from the first. It drains the V4L2 node to the newest
frame as frames land; slots that leases give back are queued again on
its thread. `wake()` makes the waiting job look at its frame's status
now -- how a cancelled long manual exposure is cut short.

The ISP engine runs one frame on the GPU (~60 ms a frame on Tegra K1,
the bottleneck); a deeper ring only adds latency and lets completions
clump. The finish engine took the CPU half off the GPU's thread, so the
ISP engine waits on nothing but fences.

## Who owns a frame

`core/pipeline/Frame.h`.

- **`Frame`** -- one request from acceptance to result: frame number,
  `FrameSettings`, the outputs (stream, buffer, acquire fence, release
  fence, status, JPEG snapshot), the Bayer lease and crop, applied
  exposure / gain, timestamps, `FrameStatus`. Fields are written by one
  engine at a time in the order the frame travels; handing a job to the
  next engine's queue orders the writes before the next reads. The
  exceptions (`hasBayer`, `status`, `inPipeline`) are atomic.
- **`FrameRef`** (`shared_ptr<Frame>`) -- held by the registry (for
  flush and close) and by the result engine until it has handed the
  result back.
- **`FrameToken`** -- a job's hold on the frame it works on; counts
  `inPipeline`. A job hands the frame on by moving its token into the
  next job; a fan-out (one JPEG job per BLOB) gives each a `share()`.
  The last token to go notifies the result engine's `settled` wake.
  **No path loses a frame:** a job that is done, refused, cancelled,
  dropped or let go by a stopping engine destroys its token, and the
  frame goes on to its result either way.
- **`FrameRegistry`** -- every frame held, by number; `cancelIf`,
  `waitEmpty`, `takeAll`. A ceiling of 16 frames: more than a framework
  keeping its stream contract (`max_buffers` 4) can send; a request
  over it goes out at once as a failed one, in order.
- **`FrameStatus`** -- Ok, cancelled or failed; set once. Every engine
  looks at it before its work and skips a frame that is not Ok. The
  finish engine *commits* a frame: from then on it completes normally,
  so no ERROR_REQUEST can follow partial results already sent.

Resources a frame holds go back by type, not by every path
remembering:

- **`BayerLease`** (`ports/FrameSource.h`) -- a hold on a V4L2 capture
  slot; the slot goes back to the capture engine's queue when the last
  lease goes. The frame's lease goes when the GPU fence signals; the
  stats cycle keeps its own for its whole span.
- **`Permit`** from a **`PermitPool`** of 2 -- the right to have one
  more frame between capture and the GPU: one on the GPU, one handed
  over and waiting. Together with the lease released at the fence this
  bounds the capture slots held past capture to three of the eight.
  The capture engine waits for a permit *before* it takes a Bayer, so
  the frame the GPU gets is the freshest one.
- **JPEG snapshots** -- a frame that goes with a snapshot still held
  gives the slot back to the ISP's ring in its destructor; a JPEG job
  dropped without encoding gives its slot up and marks its output
  failed.

## Results in order

The frame is queued on the result engine the moment it is accepted,
before any other engine sees it, and its first token is taken before
that. The result engine takes one frame at a time and does not look at
the next one until this one is out, so results go out in request order
by construction -- a JPEG that takes longer only holds back the frames
after it.

A frame that settles without having been committed did not complete:
cancelled while waiting for its Bayer, dropped by a stopping engine,
its GPU work never done. The result engine cancels it and sends it as
a failed request (`FrameOutcome::Failed` → `CAMERA3_MSG_ERROR_REQUEST`
and its buffers in error).

## Start, stop, flush, close

`Session::Impl`:

- **start** -- downstream first, so no engine hands work to one not
  yet running: frame source, result, JPEG, finish, ISP, stats,
  capture. One that fails to start stops the rest.
- **stop** -- from the source towards the result: capture, stats, ISP
  (with its 1 s grace for the fence), finish. A stopped engine lets its
  jobs go, and their tokens with them. Frames past the GPU (committed,
  waiting on a JPEG or their turn) get up to a second to go out
  normally; then JPEG and result stop, and whatever the registry still
  holds goes out as failed from the caller's thread. Then the frame
  source stops.
- **flush** -- sets `flushing`, cancels every frame in the registry,
  wakes the capture engine, waits up to a second until the registry is
  empty. Committed frames complete normally; a request arriving during
  the wait is cancelled like the rest (the flag is set before the
  registry is cancelled, so a frame added after that sees it).
- **AE manual → auto** -- apps like Open Camera switch without a flush,
  so the session cancels, through the same path, the frames still
  waiting for their Bayer (not snapshots); a long manual exposure no
  longer drains for seconds.
- **configure** -- stop, reconfigure V4L2 and the ISP, start.
- **close** -- stop, then reset what is per session: AF, ApplySettings,
  the device-error reporter, the ISP's session state, the IPA,
  DelayedControls, the stats publication.

## Device errors

`DeviceErrorReporter` sends `ResultSink::deviceError` (Camera3:
`CAMERA3_MSG_ERROR_DEVICE`) once per session, whichever part finds the
failure first: the capture engine's 3 s stall watch (no frame from the
sensor) or the ISP engine's 1 s fence watch (hung GPU). The framework then closes the camera; close stops the engines
and every frame held goes out as failed.

## DelayedControls

Port of libcamera's `DelayedControls` (`core/sensor/`): a 16-slot ring
indexed by frame number. Exposure and gain each have a delay in frames
(`SensorConfig::controlDelay`, a silicon property -- 2 for both
sensors).

- 3A on the finish engine decides exposure / gain for the frame it
  ran on and `push`es each control at `frameNumber + delay`.
- `ApplySettingsStage` on the capture engine takes the decision filed
  for its frame (`pendingWrite`), fills what the IPA did not decide
  from the last auto write, writes it (with group hold where the
  sensor has it) and `commit`s what it wrote. Manual AE, and auto
  before the first decision, write the request's values and commit
  them the same way, so a late IPA push cannot overwrite what the
  sensor really got.
- Result metadata reports `applyControls(frameNumber)`: what was in
  effect on the frame, not what its request asked.

There is no start-of-frame event on this driver, so the ring runs in
request-sequence time; dropping stale frames to the newest makes it
drift from kernel frames by the frames dropped.

## 3A and statistics

The stats engine publishes `IpaStats` every `kPhaseCount` frames; peak
CPU per frame stays about one phase's share. It subtracts the optical
black level as the demosaic shader does and restricts the focus metric
to AF's ROI, taken when the job is submitted.

3A runs once per committed frame on the finish engine
(`StatsProcessStage` → `Ipa3A::processStats`) on the latest
publication: AE, AWB and AF in that order, AWB / AE partial results to
the sink, WB gains and the CCM straight into the ISP (no sensor delay),
exposure / gain into DelayedControls, the lens through `Focuser`. AF
settings arrive from the capture engine (`onSettings`) and are applied
at the start of the next tick, so AF's state has one owner. Details in
[architecture.md](architecture.md#3a-summary).

## Produce once, sample many

The ISP engine demosaics a frame once into a scratch image; each
output is a blit from it (RGBA by sampler, NV12 by compute), all in
one command buffer and one submit. JPEG outputs take a snapshot of
the scratch into the ISP's JPEG ring, which the JPEG engine reads
after the GPU is done -- the next frame's demosaic runs beside the
encode.

## History

| Before (Tier 3) | Now |
|---|---|
| RequestThread + its queue | the capture engine's queue; settings applied on the capture engine |
| V4l2CaptureThread | `V4l2Source` driven from the capture engine's poll |
| PipelineThread, `PIPELINE_MAX_IN_FLIGHT`, completeCtx | ISP engine (GPU, fence) + finish engine (CPU half) |
| StatsWorker | stats engine, LatestOnly |
| JpegWorker | JPEG engine, the job owns the snapshot |
| ResultThread + InFlightTracker | result engine, ordered by acceptance, waits on tokens |
| PipelineContext | `Frame` + `FrameToken` + `FrameRegistry` |
| bounded blocking queues for slots | `PermitPool` + `BayerLease` |
| ThreadBase, EventQueue, Signal | `Engine` over `JobQueue` |
| NvBlit fallback | removed; one Vulkan path |

What did not change: one Vulkan queue (Tegra K1 exposes one, Vulkan
1.0, no timeline semaphores), one pipeline, the IPA in-process, and no
ZSL or reprocess (`android.request.availableCapabilities` lists
neither).

## See also

- [architecture.md](architecture.md) -- the module split, request
  lifecycle, 3A, tuning.
- [latency-and-buffers.md](latency-and-buffers.md) -- why requests are
  queued and buffers bounded.
- [open-source-references.md](open-source-references.md) -- libcamera
  and AOSP patterns drawn from.
