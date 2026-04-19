# Preview latency and V4L2 buffers

## TL;DR

On `master` the HAL is effectively pipeline-depth-1 and can present the
framework with a frame that is up to `V4L2DEVICE_BUF_COUNT - 1` frames
older than the newest frame the sensor produced. Fixing this does not
require reducing `V4L2DEVICE_BUF_COUNT`. Two compatible fixes:

1. **Drain-to-latest in `readLock()`** — keep 4 buffers but always
   return the newest ready one, requeueing stale ones immediately.
   Minimal change, ~50 lines in `v4l2/V4l2Device.cpp`.
2. **Asynchronous capture thread** — dedicate a thread to continuously
   `DQBUF → QBUF`, publishing "latest frame" via an atomic.
   `processCaptureRequest` reads latest under mutex. Larger change but
   decouples sensor cadence from HAL processing time entirely.

Also required (orthogonal) if we want accurate result metadata under
V4L2's inherent control latency:

3. **`DelayedControls`** — track which `S_CTRL(EXPOSURE=…)` is expected
   to take effect on which captured frame, so the per-frame result
   metadata reports what actually applied rather than what was
   requested.

## Why we see latency today

The kernel V4L2 queue is a ring: buffers sit in three states at any
moment — **owned by driver** (being filled by VI), **done queue** (filled,
waiting for userspace), **owned by userspace** (dequeued, in HAL).

HAL flow per request (`Camera::processCaptureRequest`, `hal/Camera.cpp:741`):

```
t0 ── DQBUF ──► process ──► QBUF ──► return ──► framework calls again ──► …
       ↑                                                       │
       └───────────── next DQBUF happens here ◄────────────────┘
```

The DQBUF at the start of each request pulls the **oldest** frame in the
done-queue by V4L2 semantics. If processing + gralloc lock/unlock +
request marshalling takes longer than a frame period, the done-queue
fills up, and the frame we dequeue is progressively older. At steady
state with 30 fps sensor and a ~40 ms pipeline, the framework sees a
frame that is ~`buf_count` frames stale.

The existing perf log line (`hal/Camera.cpp:1128`) makes this observable:

```
PERF: dqbuf=… convert=… total=…
```

If `total` exceeds one frame period on average, latency grows.

## Why lowering `V4L2DEVICE_BUF_COUNT` is not the fix

Previously attempted in commit `46524c2` (reverted). The revert message
blames Tegra VI; the actual reason is that the HAL occasionally spikes
past one frame time (GC, setControl round-trip, GPU fence wait), and
with only 2 free ring slots the driver cannot enqueue the sensor's next
frame — we lose it outright. 4 slots is a jitter cushion, not a latency
source.

Reducing the count trades latency for frame drops. Not a good trade.

## Fix #1 — drain-to-latest

After the first (blocking) DQBUF that gets us a frame, loop
non-blocking DQBUFs. Each successful non-blocking DQBUF means a newer
frame was already waiting; requeue the previous one immediately.

Sketch for `v4l2/V4l2Device.cpp:427`:

```cpp
const V4l2Device::VBuffer * V4l2Device::readLock() {
    assert(isConnected() && isStreaming());

    int id = dequeueBuffer();              // existing, blocking
    if (id < 0) return NULL;

    int dropped = 0;
    for (;;) {
        int next = dequeueBufferNonBlocking();   // new helper
        if (next < 0) break;                      // EAGAIN — none newer
        queueBuffer(id);                          // old goes back
        id = next;
        ++dropped;
    }
    if (dropped) ALOGV("readLock: skipped %d stale frames", dropped);
    return &mBuf[id];
}
```

`dequeueBufferNonBlocking()` is identical to `dequeueBuffer()` but
without the `poll()` wait and it returns `-1` immediately on
`EAGAIN` / `EWOULDBLOCK`.

**Prerequisite:** the device fd must be opened with `O_NONBLOCK`. Add
`O_NONBLOCK` to the `open()` in `V4l2Device::connect()`. The existing
blocking DQBUF path stays correct because it's preceded by a `poll()`
that waits for readiness.

**Effect:** worst-case staleness drops from `buf_count × frame_time` to
`1 × frame_time` regardless of pipeline spike size, as long as the
average pipeline time stays under `frame_time`. During a spike you still
catch up on the next request.

**Caveat:** monotonic frame-number association breaks. If the framework
expects every request to consume exactly one sensor frame (it does
assume this for timestamp/metadata correlation), dropping frames on the
driver side means `sensor_timestamp` will jump. This is acceptable for
preview but needs care if you rely on per-frame result metadata — see
fix #3.

## Fix #2 — asynchronous capture thread

Move the DQBUF/QBUF loop off the framework thread entirely.

```
  [capture thread]                 [framework thread]
  ──────────────────                ──────────────────
  while (running) {
      DQBUF → buf
      { lock; drop old mLatest;     readLatestLocked():
        mLatest = buf; signal }       { wait for mLatest;
      QBUF back the previous           take ownership;
      mLatest if any                   release mutex;
  }                                    process; QBUF back }
```

- Always one buffer in flight on userland side (`mLatest`) + N-1 in the
  driver ring. Sensor never starves.
- `processCaptureRequest` never blocks on DQBUF — it pulls the latest
  published frame.
- Skipped frames are quietly dropped in the capture thread with no
  extra QBUF round-trips on the hot path.

Cost: one thread, one mutex + condition variable, and careful shutdown
/ restart on `configure_streams`.

This is how libcamera and cros-camera both do it. See
[open-source-references.md](open-source-references.md).

## Fix #3 — `DelayedControls`

The problem fixes #1 and #2 don't solve: `VIDIOC_S_CTRL(EXPOSURE=X)`
does not take effect on the next frame dequeued. On most MIPI-CSI
sensors it takes effect 2–3 frames later (sensor pipeline latency).
So the result metadata returned to the framework reports "I just set
exposure X", but the frame we return was actually captured with the
previous exposure value.

libcamera solves this with `DelayedControls`: a per-control ring that
records `{frame_number, pending_value}`, parameterised by the delay
value for each control (typically 2 frames for exposure, 2 for gain,
1 for VCM). On each captured frame, pop the entry whose delay has
elapsed — that's the value that actually applied, and that's what you
put in result metadata.

Sketch:

```cpp
class DelayedControls {
public:
    void push(uint32_t frameNum, int32_t exposure, int32_t gain, int32_t vcm);
    AppliedValues pop(uint32_t frameNum);   // whatever was pushed
                                            // `delay_frames` earlier
private:
    struct Entry { uint32_t frameNum; int32_t exp; int32_t gain; int32_t vcm; };
    std::deque<Entry> mHistory;
    const int mExposureDelay = 2;
    const int mGainDelay     = 2;
    const int mVcmDelay      = 1;
};
```

Without this, `LIMITED` manual-exposure sequences ("set EV=-2, capture
3 frames, set EV=+2, capture 3 frames" — standard HDR bracketing) produce
wrong results and apps reject the output.

## Sensor-level latency is separate

Beyond V4L2 queueing there is another latency floor:

1. **Sensor exposure** (up to `exposure_time` itself — 33 ms at 1/30).
2. **MIPI transfer** (~1 frame period).
3. **Tegra VI capture** (~1 frame period).

This gives a *physical* minimum latency of ~3 frame periods from "photon
hits pixel" to "first byte readable by userspace", regardless of HAL
software. Any preview-latency improvement beyond that requires sensor
/ VI tuning, which is outside this HAL.

## Measurement

All three fixes are worth testing in isolation. Measurement tools:

- Existing `BENCHMARK_SECTION` / `PERF:` log. Baseline `total` before
  fixes; watch it after.
- Read `buf.timestamp` from `v4l2_buffer` (currently discarded in
  `V4l2Device::dequeueBuffer`) and log `systemTime() - timestamp` to
  see the real age of each dequeued frame.
- `shutter` → `displayed` latency is only observable with an external
  camera recording a flashing LED or running `ScreenRecord` with frame
  timestamps.
