# Open-source references

What other open camera stacks do, and which of their patterns are worth
stealing into this HAL. All links are to external projects — none of
their code is vendored here.

## libcamera (main reference)

<https://libcamera.org> — modern Linux camera framework. Think of it as
"what a Camera HAL would look like if designed from scratch in the
2020s". The Android-shim layer `src/android/` inside libcamera is
literally a Camera3 HAL backed by libcamera.

Patterns directly relevant to this HAL:

### Request objects detached from buffers

`libcamera::Request` (<https://libcamera.org/api-html/classlibcamera_1_1Request.html>)
carries stream → buffer bindings and per-frame control updates. Apps
queue multiple Requests ahead. The pipeline handler pulls them from a
queue, binds V4L2 buffers on-demand, and completes them in submission
order.

**Steal:** a `CaptureRequestQueue` between `processCaptureRequest`
(producer) and the actual V4L2/ISP/gralloc pipeline (consumer). Breaks
the 1-in-flight limitation.

### `PipelineHandler`

<https://libcamera.org/guides/pipeline-handler.html>

Per-SoC subclass that knows the media-controller graph: which V4L2
subdevs exist (sensor, CSI, ISP, resizer), how to link their pads, how
to configure formats end-to-end. The "flat" V4L2 API this HAL uses
works only because Tegra exposes a pre-linked video node (`/dev/video0`).
For anything more complex (multiple ISP stages, statistics readback,
separate capture and stats video nodes) you need media-controller.

**Steal:** even without adopting libcamera proper, moving sensor / ISP
subdev access from `/dev/video0`-only to `/dev/media0` + explicit
subdevs unlocks per-stage format negotiation and statistics queues.

### `DelayedControls`

<https://libcamera.org/api-html/classlibcamera_1_1DelayedControls.html>

Ring buffer tracking which control value maps to which captured frame,
parameterised by per-control delay. Covered in detail in
[latency-and-buffers.md](latency-and-buffers.md).

**Steal:** straight port. ~150 lines, zero dependencies on rest of
libcamera.

### IPA (Image Processing Algorithm) module

<https://deepwiki.com/raspberrypi/libcamera/3.1-ipa-architecture>

3A algorithms live in a separate module fed by a **statistics stream**
from the ISP (histograms, AF sharpness grid, AWB patch averages). It
returns control updates for the next frame. On ChromeOS the IPA runs in
a sandboxed process with Mojo IPC for security; on embedded it's
in-process.

**Steal:** even without the sandbox, separating 3A into its own
`IpaInterface { process(stats) → controls }` is a clean win. Makes AF /
AE / AWB testable in isolation, allows swapping algorithms per sensor,
and ends the "AF reads the rendered preview" kludge in this HAL.

### Reference pipelines close to Tegra in spirit

- **RkISP1** — <https://github.com/libcamera-org/libcamera/tree/master/src/libcamera/pipeline/rkisp1>.
  Rockchip ISP on RK3288 / RK3399 — same generation as Tegra K1,
  similar media-controller topology (sensor → ISP → separate capture
  and statistics video nodes).
- **IPU3** — <https://github.com/libcamera-org/libcamera/tree/master/src/libcamera/pipeline/ipu3>.
  Intel's ISP. More complex graph, but the request pipelining and
  statistics flow are canonical references.

## Raspberry Pi libcamera fork

<https://github.com/raspberrypi/libcamera>

RPi maintain their own IPA in `src/ipa/rpi/`. The interesting bit is
**tuning files**: per-sensor JSON with every knob the algorithms read
(AWB colour gains at each colour temperature, AEC curves, AF macro
focus distance, CCM matrices, lens-shading tables).

**Steal:** a JSON tuning file loaded at HAL init, keyed by sensor model.
Replaces every hardcoded constant in this HAL (`mSensorCfg`, VCM
ranges, focal length, sensor area, colour transform) and makes adding a
second sensor a data-only change rather than a code change.

Docs: <https://www.raspberrypi.com/documentation/computers/camera_software.html>

## ChromeOS `cros-camera`

<https://chromium.googlesource.com/chromiumos/platform2/+/HEAD/camera/>

Reference HAL3 used on Chromebooks with Intel IPU3 / IPU6. Patterns:

- **Request manager** — explicit request-queue abstraction with
  ordered result delivery. Worth studying even if you don't adopt it
  wholesale.
- **PSL (Platform-Specific Layer)** — per-chipset hardware glue behind
  a stable upper interface. Maps cleanly to `IspPipeline` here —
  arguably `IspPipeline` is already a proto-PSL.
- **Mojo IPC to sandboxed 3A** — overkill for embedded, but
  demonstrates that 3A really doesn't need to share a process with
  hardware-facing code. Useful if / when `libnvisp_v3` proves flaky
  (a crash in the blob shouldn't take down the HAL).

## AOSP V4L2 camera HAL

`hardware/libhardware/modules/camera/3_4/` in the AOSP tree. USB-webcam
oriented, pure V4L2, no ISP — closest architectural analogue to this
HAL.

Shows how far you can go with pure V4L2:

- Three-stage pipeline: requests accepted → enqueued for V4L2 → dequeued
  async → results emitted. Runs on a dedicated request thread.
- Best-effort per-frame control application.
- Simple metadata class organising static / request / result keys.

**Steal:** the three-stage threading model is a lighter alternative to
the full libcamera approach if we want to stay within our existing
C++11 / AOSP build.

## AOSP `ExternalCameraDevice`

`hardware/interfaces/camera/device/3.x/default/` in AOSP. Official HIDL
HAL for USB UVC cameras. Also V4L2-based, more modern than the
`libhardware` version. Good reference for the bits the old V4L2 HAL
gets wrong (error callbacks, stream reconfiguration, capability
reporting).

## Android emulator camera HAL

`device/generic/goldfish/camera/` — the `qemud` / `EmulatedFakeCamera3`
in AOSP. Interesting because it's the **reference the Camera team uses
to validate the framework** — it implements most Camera3 behaviours
deliberately correctly, backed by synthesised pixels. If you're unsure
"what's the expected AE state machine", reading `EmulatedFakeCamera3`
answers that.

## Tegra K1 / T124 specifics

There is no publicly maintained open stack for Tegra K1 / T124 cameras.
Fragments worth knowing about:

- **L4T (Linux for Tegra)** — <https://developer.nvidia.com/embedded/tegra-k1>.
  Ubuntu-derived distribution for Jetson TK1. Includes kernel sources
  (`tegra_camera_platform`, `vi`, `isp-t124`) but no userspace ISP stack.
- **`nvhost-isp` / `libnvisp_v3`** — the Tegra ISP-A userspace library
  used via blob in this HAL. Closed-source. No publicly documented
  calibration format. Tuning comes from pre-built binary blobs.
- **Jetson Multimedia API / Argus** — NVIDIA's proprietary userspace
  camera API. Not open. But the API surface
  (<https://docs.nvidia.com/jetson/l4t-multimedia/index.html>) is
  informative about the sort of IQ pipeline NVIDIA wraps around ISP-A.

## Short list — what to actually study first

If time is limited, in priority order:

1. **libcamera's RkISP1 pipeline handler** — closest architectural
   analogue. Read top-to-bottom.
2. **libcamera `DelayedControls`** — small, self-contained, directly
   portable.
3. **RPi IPA + tuning JSON** — pattern for separating algorithm from
   per-sensor data.
4. **AOSP `EmulatedFakeCamera3`** — reference for correct Camera3
   metadata and state-machine behaviour.
5. **AOSP V4L2 HAL `3_4`** — minimum-viable async V4L2 request queue.
