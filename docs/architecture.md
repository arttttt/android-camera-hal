# Architecture

The HAL is two modules: a camera **core** that knows the sensor, the
ISP, 3A and JPEG but no framework interface, and a thin **Camera3
frontend** that translates between the core and Android's
`camera3_device`. The execution model inside the core -- its threads,
how a frame travels and is owned, cancellation and shutdown -- is in
[tier3_architecture.md](tier3_architecture.md).

```
 Android camera provider (HIDL 2.4)
        │ dlopen camera.tegra.so, camera3_device ops
        ▼
 hal/  camera.tegra  ── Camera3 frontend ───────────────────────────────
   HalModule          enumerates /dev/video*, one Camera3Device each
   Camera3Device      the camera3_device: ops, stream table, templates
   Camera3Request     metadata → FrameSettings (once, on acceptance)
   Camera3ResultSink  ResultSink → notify / process_capture_result
   Camera3GrallocPort GrallocPort → GraphicBufferMapper
   StreamConfig, metadata/ (static characteristics, templates, results)
        │ mochacamera/*.h only: Session, FrameRequest, FrameSettings,
        │ Capability, ResultSink, GrallocPort, NativeBuffer
        ▼
 core/ libmochacamera  ── the camera core ─────────────────────────────
   session/   Session (facade), CapabilityProbe
   engines/   Capture, Stats, Isp, Finish, Jpeg, Result
   engine/    Engine, JobQueue, PermitPool
   pipeline/  Frame, FrameRegistry, FrameStatus, stages, BufferProcessor
   3a/ ipa/   AE, AWB, AF, Ipa3A, NEON stats
   isp/       Vulkan ISP backend (demosaic, blits, NV12, JPEG ring)
   jpeg/      libjpeg encoder with EXIF + thumbnail
   sensor/    SensorConfig, SensorTuning, DelayedControls
   v4l2/      V4l2Device, V4l2Source (FrameSource), Focuser, MediaGraph
        │
        ▼
 kernel: tegra_vi V4L2, IMX179 / OV5693 sensors, VCM subdev; nvhost GPU
```

## The boundary

The core speaks its own vocabulary, in `core/include/mochacamera/`:

- **Controls.h** -- the 3A enums (`AeMode`, `AeState`, `AwbMode`,
  `AwbState`, `AfMode`, `AfTrigger`, `AfState`, `CaptureIntent`) with
  Camera2's values set explicitly, plus `SensorRect`, `CropRegion`,
  `MeteringRegion`, `Rational`, `Size`.
- **FrameSettings.h** -- a request as the core reads it. Parsed once
  by the frontend (`parseFrameSettings`); absent keys get Camera2's
  values, or no value where absence means something (default
  exposure, no thumbnail, centre metering).
- **Results.h** -- `Core3AResult` (the AE / AWB / AF states and the
  sensor values in effect), `CoreFrameResult`, `PartialResult`.
- **Buffers.h** -- `NativeBuffer` (the frontend's buffer, opaque: with
  Camera3 it is the request's `buffer_handle_t*`), `StreamSpec`,
  `PixelFormat` (RGBA, YCbCr 4:2:0, BLOB), `BufferReturn`.
- **Capability.h** -- what the camera can do; see below.
- **Session.h** -- the facade: `open`, `close`, `configure(Size)`,
  `running`, `submit(FrameRequest)`, `flush`, `dump`, `capability`.
- **ports/** -- what the core calls out through: `ResultSink`
  (shutter, partial, frame, deviceError), `GrallocPort` (CPU lock /
  unlock of output buffers), and `FrameSource` + `BayerLease` +
  `BayerFrame` (the capture source, implemented by `v4l2/`).

Two invariants keep the boundary honest:

- **Nothing under `core/` includes `camera3.h` or `CameraMetadata`.**
  `core/tests/run_host_tests.sh` checks it; the Android build would
  not find `CameraMetadata` for the core anyway, since only the
  frontend links the camera-metadata helper.
- **The frontend sees only `core/include`.** `libmochacamera` exports
  that directory alone, so `hal/` cannot reach the core's internals.

The enums are converted with a cast; `hal/Camera3Request.cpp`
`static_assert`s every member against its `ANDROID_CONTROL_*` value.
Where Camera3 has more modes than the core implements, the parser maps
them onto the core's (any auto-exposure mode is `AeMode::On`).

## Request lifecycle

1. **Acceptance** (binder thread, `Camera3Device::processCaptureRequest`).
   Outputs are mapped to the core's `StreamSpec` through the stream
   table (an unconfigured stream is `-EINVAL`); acquire fences pass to
   the core; settings are parsed (a request without settings repeats
   the last ones); a BLOB output makes it a snapshot. The result sink
   is told to expect the frame -- it keeps the request's metadata as
   the echo the results start from -- and `Session::submit` takes it.
2. **The core** (see [tier3_architecture.md](tier3_architecture.md)):
   the capture engine applies the settings, notifies the shutter and
   takes the Bayer; the demosaic engine records and submits the GPU
   work; the finish engine commits the frame, finishes the YUV outputs
   on the CPU, runs 3A (which sends the AWB and AE partial results)
   and hands BLOB outputs to the JPEG engine; the result engine hands
   the frame back, strictly in submission order.
3. **Results** (`Camera3ResultSink`). Shutter → `CAMERA3_MSG_SHUTTER`.
   Partial results → `process_capture_result` with counters 1 (AWB)
   and 2 (AE), metadata = request echo + that subset. The final result
   → counter 3 (`kPartialResultCount`) with all result keys and the
   buffers; an output that failed gets `CAMERA3_MSG_ERROR_BUFFER`
   first. A failed or cancelled frame is `CAMERA3_MSG_ERROR_REQUEST`
   followed by its buffers in error, without metadata (partial 0).

`flush()` runs without the device mutex (Camera3 lets
`process_capture_request` run beside it): the core cancels every frame
it holds and waits up to a second until none is left; committed frames
complete normally.

## Stream configuration

`StreamConfig::normalize` validates the stream list (no rotation, no
depth, no ZSL usage, one input at most), maps
`IMPLEMENTATION_DEFINED` to RGBA (or YCbCr for a video encoder
consumer), and sets usage bits and `max_buffers` (4).

`Camera3Device::configureStreams` then builds the stream table
(`StreamId` = index in this configuration) and hands the outputs to
`Session::configure`, which picks the sensor mode
(`chooseCaptureSize`, among the sizes the capability lists):

- a YUV output is the capture itself -- the NV12 encoder does not
  scale (`Capability::yuvAtCaptureSizeOnly`) -- so it names the mode,
  and every other output must fit in it;
- otherwise the mode already running is kept when it is at least as
  wide and as tall as every output, since RGBA outputs scale in the
  blit and JPEG outputs in the encoder, each from its own centred part
  of the crop (`cropForOutput`);
- otherwise the smallest mode that is.

Outputs no mode feeds are refused (`BAD_VALUE`) before anything
changes. Keeping the running mode is what lets apps switch stream sets
without a restart: Open Camera reconfigures on every AE auto↔manual
toggle, and after a recording stops it configures one 320×240 RGBA
output that no sensor mode has. A mode change stops the engines (every
frame held goes back first), reconfigures V4L2 and the ISP (input
buffers exported to V4L2 as DMA-BUFs), and starts them again -- ~3 s
on Tegra K1.

## Capability, static characteristics, templates

`probeCapability` (core) gathers what the camera can do, once per
camera: the sensor and lens (driver sizes; tuning physical size, focal
length, minimum focus distance, orientation, colour filter), exposure
and sensitivity ranges from the driver, the output sizes the SW ISP
sustains (16:9 up to 1080p, `OutputResolutionCap`) in each format with
their frame durations, AE / AWB / AF modes (AWB presets only with the
Bayesian AWB and a tuning that maps them), the EV step and range, FPS
ranges from the driver's frame intervals, JPEG buffer size and
thumbnail sizes, pipeline depth and output counts.

The frontend only translates it: `CameraStaticMetadata::build` writes
the tags, adding what is Camera3 policy rather than capability (the
per-stage OFF modes, the hardware level LIMITED, the key lists);
`RequestTemplateBuilder` reads the active array (crop, regions) and
the AF modes (Auto where the lens moves).

## 3A summary

3A runs on the finish engine, once per frame, on the latest statistics
the stats engine published (`Ipa3A::processStats`).

- **AE** (`AutoExposureController`). State is `filteredTotalUs`
  (exposure × gain at unity, µs). Each frame two candidate ratios:
  `ratioMean = setpoint / lumaInRoi` (spot mean of the green channel
  inside the focus ROI, setpoint `pow(MeanAlg.target/255, 2.2)`) and
  `ratioHighlight = highlightCap / IQM_top2%` (post-WB max-of-channels
  of the brightest ~2 % of patches). Strictest wins, clamped to the
  per-sensor `MaxFstopDelta` rate limit; a single-pole LPF moves
  `filteredTotalUs` toward the target; a 2 % dead-band freezes it at
  convergence. EV compensation multiplies the target by
  2^(steps × `Capability::ae.evStep`). AE_LOCK holds the converged
  point with an EMA-smoothed bias so EV steps under lock don't tear
  the rolling-shutter readout. Exposure / gain split via
  `SensorConfig::splitExposureGain` (exposure up to one default frame,
  then gain). Decisions go to `DelayedControls` at the frame they land
  on; the capture engine's `ApplySettingsStage` writes them.
- **Manual AE** (`AeMode::Off`): `parseManualSettings` takes the
  request's exposure and sensitivity verbatim, grows the frame for a
  long exposure, and the same EV scaling applies.
- **AWB** behind the `Awb` interface, picked per sensor by
  `AwbFactory` from the tuning's `active.awb.algorithm`:
  *GrayWorldAwbController* (gray-world over `rgbMean[16][16]` with
  patch filtering, a 96-patch confidence gate relaxing to the
  calibrated daylight prior; OV5693) and *BayesianAwbController*
  (RPi-style search over calibrated CT curves with a lux-conditioned
  prior, bias term, transverse fine search and per-mode CT windows for
  presets; IMX179). WB gains apply in the demosaic shader with no
  silicon delay; the CCM is interpolated between calibrated CCT
  brackets into the buffer the shader reads. See
  [awb-bayes.md](awb-bayes.md).
- **AF** (`AutoFocusController`): a contrast-detect sweep
  (`Coarse1 → [Coarse2] → Fine → Settle`, parabolic peak
  interpolation) behind the Camera2 AF state machine (Inactive /
  PassiveScan / PassiveFocused / PassiveUnfocused / ActiveScan /
  FocusedLocked / NotFocusedLocked). Settings are queued from the
  capture engine and applied at the start of the next tick, so AF's
  state has one owner. Continuous-picture mode retriggers on scene
  change; a sweep samples only fresh statistics of a frame exposed
  after the lens settled. AE and AWB are held for the length of any
  sweep (a level, not edges). Tap-to-focus takes the request's AF
  region (one, `CONTROL_MAX_REGIONS = {0, 0, 1}`), grown to a minimum
  patch footprint; the same ROI restricts the NEON focus metric and
  weights AE / AWB metering. The lens is written by the IPA through
  `Focuser` on the same tick AF decides it.
- **States** (`describe3A`, as Camera2's state tables have them): AE
  Off → Inactive, locked → Locked, converged → Converged, else
  Searching. AWB Auto → Locked while the request locks it, else
  Converged or Searching as the controller judges it
  (`AwbConvergence`: the signed gap between the smoothed gains and
  the tick's target, averaged over ~5 ticks so the Bayes search noise
  cancels, within 2 %, out again past 5 %); holding AWB through an AF
  sweep keeps the state it had. Off and the presets → Inactive. The same
  derivation feeds the partial and final results.

## Configuration knobs

- `V4L2DEVICE_BUF_COUNT` (compile time, `Android.bp`) -- the V4L2
  capture ring, 8.
- `persist.camera.soft_isp` (runtime, read by `Camera3Device` once per
  camera) -- `1` (default): full ISP (WB, CCM, gamma) and 3A; `0`:
  demosaic only, kept for cold bring-up.
- Which sensor and module each camera is (`Camera3Device`): back =
  IMX179 by Primax, front = OV5693 by Sunny; HalModule assigns facing
  by enumeration order.

## Per-module tuning files

`/vendor/etc/camera/tuning/<lower(sensor)>_<lower(integrator)>.json`,
converted from the stock NVIDIA `.isp` overrides by
`tools/isp_to_json.py`, installed by the `prebuilt_etc` modules in
`Android.bp`. Sections:

- `active` -- keys with a live consumer: `af.*` (VCM range, settle
  time), `colorCorrection.Set[]` (multi-CCT CCM + neutral priors),
  `opticalBlack.*`, `ae.MeanAlg`, `awb.v4.*`, `awb.bayes.*`.
- `active.hal_overrides` -- HAL knobs with no `.isp` counterpart,
  preserved verbatim across regenerations: `ae.close_speed_zone`, and
  the contrast-AF knobs under `af` (`step_coarse`, `step_fine`,
  `contrast_ratio`, `retrigger_ratio`, `retrigger_delay_frames`,
  `pdaf_enabled`, `min_frames_between_sweeps`,
  `min_focus_roi_patches`; all required -- a missing one disables AF).
- `reserved` -- the rest of the stock tuning, kept 1:1 until a stage
  consumes it.
- `module` -- datasheet values not in the `.isp` (physical size, focal
  length, min focus distance, orientation, bayer pattern), merged from
  `tuning/_module_<sensor>_<integrator>.json`.

`SensorTuning` loads in the `Session` constructor; without it the core
runs on compile-time defaults (and AF stays off).

## Build and tests

`Android.bp` builds `libmochacamera` (static, `-Wall -Wextra -Werror`,
C++20) and `camera.tegra` (the frontend, linking it), plus the tuning
files and two diagnostic probes. From a LineageOS 19.1 tree:

```
mmm hardware/camera
```

The core's host tests (JobQueue, Engine, PermitPool, Frame and its
holders, DelayedControls, the result engine on a fake sink, 3A on
synthetic statistics) run under ThreadSanitizer from the repository
root, in the build tree:

```
core/tests/run_host_tests.sh
```
