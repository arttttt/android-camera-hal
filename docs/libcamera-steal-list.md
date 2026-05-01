# What to borrow from libcamera

Concrete, ranked shopping list of patterns and code from libcamera worth
pulling into this HAL. Compiled from a dimension-by-dimension survey of
both trees against current state (~16 KLOC, single SoC, in-process 3A,
Vulkan ISP).

Different from [open-source-references.md](open-source-references.md):
that one is a broad survey of upstreams; this one is a scoped TODO list
with file paths on both sides and the concrete bug or limitation each
item resolves. Items are framed as "lift the pattern", not "vendor the
code" — libcamera's licence is LGPL-2.1 and this HAL ships under MIT.

Reference paths below assume `libcamera/` checked out at
`/Users/artem/Projects/libcamera`.

## Ranking by impact-per-effort

Image-quality wins (directly resolve `bugs.md` items or close known
algorithmic gaps):

1. **Lens-shading correction (ALSC)** — fixes the AE-overbrightens bug
2. **Per-frame `IspParams` struct** on the ISP boundary — Tier 3 enabler
3. **Sensor-properties database pattern** — kills hard-coded
   `lineTimeUs` / `controlDelay` in `SensorConfig`
4. **Bayesian AWB with lux-dependent priors** — beats gray-world in
   mixed light

Architectural cleanups (pay off when a second sensor / SoC lands, or
when 3A needs to be swapped):

5. **Explicit `Fence` type** on the buffer boundary
6. **`IpaInterface` factoring** without IPC — clean swap for `BasicIpa`
7. **MediaDevice / subdev introspection** — replaces `/dev/video0` scan
8. **Multi-channel AGC** for HDR-region metering
9. **Color-space awareness** beyond hardcoded sRGB
10. **Noise-model section** in tuning JSON
11. **Pixel-level histogram + iterative saturation-aware gain** — refines
    AE highlight protection beyond patch-grid resolution

## 1. Lens-shading correction (ALSC)

**In libcamera:** `src/ipa/rpi/controller/rpi/alsc.cpp` (869 LOC) plus
per-CCT lens-shading tables in `src/ipa/rpi/vc4/data/<sensor>.json`
under the `rpi.alsc` section. RkISP1 has a hardware-fed equivalent
(`src/ipa/rkisp1/algorithms/lsc.cpp` + `data/`), simpler algorithm but
the same per-CCT table shape.

**Where it would land:** new shader pass in `isp/vulkan/shaders/`
applied between WB and CCM (it's a per-pixel multiplicative mask,
cheaper to fold into the existing demosaic compute than a separate
dispatch). Tuning section under a new `lensShading` key alongside
`colorCorrection.Set[]` in `tuning/<sensor>_<integrator>.json`. New
loader in `isp/sensor/SensorTuning.cpp`.

**Addresses:** `bugs.md` "AE over-brightens preview" — that note
identifies LSC as the biggest lever. Also kills the off-axis
brightness bias that AE chases when the FocusRoi sits in a vignetted
corner.

**Effort:** medium. The shader change is small; the work is the
**calibration session** to produce per-CCT correction tables for
IMX179 and OV5693 (a flat-field shoot under a few illuminants, then
fit a low-order surface). RPi documents the procedure; their
`ctt_lacalibration.py` is a reasonable reference.

**Caveats:** ALSC tables are ~1.5 KB per CCT × ~5 brackets per
sensor; embedding them in the JSON tuning is fine. Algorithm itself
is < 200 LOC of HAL-side code, the heavy 869 LOC of RPi `alsc.cpp`
covers the **adaptive** LSC variant — start with static per-CCT
tables, defer adaptive.

## 2. Per-frame `IspParams` struct on the ISP boundary

**In libcamera:** every pipeline handler computes per-frame ISP params
via `ipa_->computeParams(frame, paramBuf->cookie())`
(`src/libcamera/pipeline/rkisp1/rkisp1.cpp:1343`). Params land in a
shared buffer the ISP reads at consume time. The IPA's
`processStats(stats) → controls` returns metadata the next
`computeParams` rolls in.

**Where it would land:** already on the Tier 3 roadmap (see
[tier3_architecture.md](tier3_architecture.md), "Per-frame ISP tuning
channel"). Today `VulkanIspPipeline::process*` takes only geometry;
WB/CCM/gamma are mutated via `setCcm` / `setWbGains` between
submits. Lift this to a per-frame `IspParams` populated by `BasicIpa`
and consumed by `DemosaicBlitStage`. The struct already exists
in shape — `isp/vulkan/IspParams.h` — but is set as a singleton, not
threaded through the request.

**Addresses:** the "per-frame ISP tuning channel" item in
`tier3_architecture.md`. Unblocks per-request CCM/WB without TOCTOU
between request N and request N+1.

**Effort:** small once the request descriptor (`PipelineContext`)
already carries the slot. ~150 LOC plus shader binding update.

## 3. Sensor-properties database pattern

**In libcamera:** `src/libcamera/sensor/camera_sensor_properties.cpp`
(442 LOC) — a flat C++ map from sensor model name → struct
{`unitCellSize`, `testPatternModes`, `sensorDelays {exposure, gain,
vblank}`, ...}. ~30 sensors covered. Used as fallback when V4L2 doesn't
expose detail.

**Where it would land:** `isp/sensor/SensorPropertiesDb.{h,cpp}`. Today
the same data lives across `SensorConfig::imx179()` / `::ov5693()`
factory functions in `isp/sensor/SensorConfig.h` (hardcoded
`lineTimeUs`, `controlDelay`, `maxCoarseDiff`). Pulling it into a single
table makes adding a third sensor **a row in the table**, not a new
factory function.

**Addresses:** the "hard-coded vs tuning-driven" split in our sensor
layer — moves more of the per-sensor metadata to data, fewer
recompiles.

**Effort:** small. Mechanical refactor of two existing factory
functions plus a name → row lookup at openDevice. ~100 LOC.

## 4. Bayesian AWB with lux-dependent priors

**In libcamera:** `src/ipa/rpi/controller/rpi/awb_bayes.cpp` (444 LOC) +
`awb.cpp` (483 LOC). Tuning carries a piecewise-linear prior:
`P(CCT | lux)` lookup per illumination level. The estimator combines
gray-world stats with the prior to pick gains under low-confidence
scenes (which is exactly when our gray-world fails).

**Where it would land:** `hal/ipa/BayesianAwb.{h,cpp}`, alongside the
existing gray-world path in `BasicIpa`. The 96-valid-patch confidence
gate we already have (see memory: BasicIpa AWB design) becomes the
trigger for falling back to the prior — except instead of EMA-relaxing
to `wbGainPrior` (a static daylight neutral), we relax to the
lux-conditioned prior from tuning.

**Addresses:** mixed-light scenes where gray-world systematically
biases (warm-light bedroom + window) — currently the EMA-relax pulls
to a neutral that's wrong for the scene. The lux-conditioned prior
lets us pull to "what's plausible at this brightness level".

**Effort:** medium. The algorithm is contained, but tuning needs a
calibration session per sensor (shoot ColorChecker under a half-dozen
illuminants at different EV levels). The schema additions to
`tuning/<sensor>.json` go under a new `awb.bayes` section adjacent
to existing `awb.v4`.

**Caveats:** RPi's NN-AWB variant (`awb_nn.cpp`, 435 LOC) is **not**
worth porting — the inference dependency is heavier than the gain
in IQ on our class of sensor.

## 5. Explicit `Fence` type on the buffer boundary

**In libcamera:** `include/libcamera/fence.h` (32 LOC) +
`src/libcamera/fence.cpp` (138 LOC). Wraps a `UniqueFD`, exposes
`isValid()` / `fd()`, lifetime tied to the `Request` it's attached to.

**Where it would land:** `base/Fence.{h,cpp}`. Today the sync_fence fd
emitted by `vkQueueSignalReleaseImageANDROID` is plumbed straight into
`camera3_stream_buffer.release_fence` as a raw int in
`hal/pipeline/stages/DemosaicBlitStage.cpp`. Wrapping it gives:
explicit ownership transfer to the framework, debuggable lifetime
(currently a leak here is silent), and a clean place to hang
acquire-fence-wait logic when reprocessing eventually arrives.

**Addresses:** no current bug, but it's the right shape before the
buffer-handling layer gets touched again for the produce-once-multi-out
work in Tier 3.5.

**Effort:** small. ~80 LOC type + mechanical replacement of `int fd`
plumbing.

## 6. `IpaInterface` factoring without IPC

**In libcamera:** `include/libcamera/ipa/ipa_interface.h` (27 LOC) +
the per-pipeline Mojo IDL (e.g. `include/libcamera/ipa/rkisp1.mojom`).
What's worth taking is the **interface shape**, not the IPC machinery:
`init / configure / queueRequest / processStats / metadataReady`.

**Where it would land:** we already have `hal/ipa/Ipa.h` with
`processStats() → DelayedControls::Batch`. Lift it to a fuller
interface that also owns the per-frame `IspParams` produced (item 2)
and the result-metadata contributions (AE state, AF state). Both
`BasicIpa` and `StubIpa` already exist as concrete impls — this is
about widening the contract, not introducing the boundary.

**Addresses:** today `BasicIpa` reaches into `AutoFocusController` for
AE-lock coordination via direct calls. A widened interface with
explicit "before-frame" / "after-stats" hooks makes the dependency
direction one-way and testable in isolation.

**Effort:** small to medium. Interface widening (~50 LOC) plus
threading the new return values through the pipeline stages. Skip
the Mojo / process-isolation half — see "Not worth borrowing" below.

## 7. MediaDevice / subdev introspection

**In libcamera:** `src/libcamera/media_device.cpp` (754 LOC),
`v4l2_subdevice.cpp` (1847 LOC), `media_pipeline.cpp` (313 LOC). Walks
`/dev/media*`, enumerates entities and pads, links them, configures
formats end-to-end.

**Where it would land:** `v4l2/MediaDevice.{h,cpp}` plus
`v4l2/V4l2Subdev.{h,cpp}`. Today `hal/HalModule.cpp:43-96` scans
`/dev/video0..15` and assumes one capture node per camera; sensor +
focuser are touched separately via `V4l2Device::openFocuser()` that
takes a hardcoded path. Anything beyond Tegra's pre-linked node graph
is opaque to us.

**Addresses:** zero current bugs (Tegra K1's V4L2 driver collapses the
graph). But it's the blocker for **portability** — the moment a second
SoC enters the picture, the `/dev/video0` assumption falls over.

**Effort:** medium-large. The libcamera implementation is mature and
worth studying carefully; a stripped-down version covering "discover
subdevs, configure formats along a known chain" is ~600 LOC. Defer
until there's a concrete second-target use case.

## 8. Multi-channel AGC

**In libcamera:** `src/ipa/rpi/controller/rpi/agc.cpp` (344 LOC) +
`agc_channel.cpp` (1003 LOC). Lets multiple AE "channels" (e.g.
short-exposure + long-exposure for HDR composition, or
spot+matrix metering) run in parallel and feed back independent gains.

**Where it would land:** `hal/ipa/MultiChannelAgc.{h,cpp}` alongside
`BasicIpa`. Today AE is single-channel proportional in `BasicIpa::
processStats`.

**Addresses:** no current bug — but anything HDR-shaped (bracketed
exposure stacks for snapshot, or split-readout sensors) needs this
shape. Mi Pad 1 sensors don't gain from it directly; this is on the
list for shape-correctness, not immediate need.

**Effort:** medium-large. Skip until there's a use case
(bracketed-snapshot mode for JPEG, etc.).

## 9. Color-space awareness beyond hardcoded sRGB

**In libcamera:** `src/libcamera/color_space.cpp` (567 LOC) — enum
{sRGB, Rec709, Rec2020, DCI-P3, RAW} carrying primaries, transfer
function, Y'CbCr encoding. Stored on `StreamConfiguration`.

**Where it would land:** `hal/metadata/ColorSpace.{h,cpp}`. Today the
sRGB gamma LUT is baked into the demosaic shader via
`IspParams::gammaLut[256]` and there's no per-stream variation. Adding
the **enum and per-stream tag** in metadata is the cheap part; honoring
Rec709 transfer for video-recording streams is the right thing to do
even if we never ship Rec2020.

**Addresses:** correctness for video-recording streams that downstream
encoders expect Rec709. Currently we feed them sRGB-gamma-encoded
pixels and they trust the tag — which we don't even set.

**Effort:** small for the metadata + tag (~80 LOC). Medium if we
swap the gamma LUT per stream (need either two demosaic dispatches or
a deferred per-output gamma pass).

## 10. Noise-model section in tuning JSON

**In libcamera:** RPi tuning carries a per-sensor noise model under
`rpi.noise` in `src/ipa/rpi/vc4/data/<sensor>.json` (variance
coefficients vs gain). Drives the denoise algorithm.

**Where it would land:** new `noise` key in
`tuning/<sensor>_<integrator>.json`, parsed into `SensorTuning::
NoiseModel`. Today no denoise stage exists; this is the **prerequisite
for adding one**, since a tuned denoiser needs noise variance vs gain
to set its strength sanely.

**Addresses:** "gain-dependent colour cast at high analog gain"
(`bugs.md`) is partly noise floor crossing the WB statistics; a noise
model lets the IPA discount low-SNR patches in the gray-world average.

**Effort:** small for the schema + loader (~60 LOC). The denoiser
itself is a separate piece of work.

## 11. Pixel-level histogram + iterative saturation-aware gain

**In libcamera:** mainline `src/ipa/libipa/agc_mean_luminance.cpp`
runs an inner fixed-point iteration in `estimateInitialGain()` (up to
8 passes), each pass calling vendor-specific `estimateLuminance(yGain)`
that **clip-saturates per-cell at 255 before averaging**. Gain
converges on a self-consistent value where highlights are already
modelled as saturating — the inner loop's non-linearity is what makes
the controller self-damping near clipping. RPi `agc_channel.cpp` has
the same pattern, plus `interQuantileMean(qLo, qHi)` operating over
a real `Histogram` (`src/ipa/rpi/controller/histogram.cpp`) of
per-pixel Y bins.

**Where it would land:** `hal/ipa/NeonStatsEncoder` gains a Y
histogram output (~128 bins of green-channel post-OB pixel counts).
`BasicIpa::top2pcMaxPostWbMean` becomes `interQuantileMean` against
the real histogram instead of the 16×16 patch grid; ROI scoping
stays. Optionally an iterative gain estimator wraps the existing
`ratio = setpoint / luma` loop with up to N passes per frame, each
applying the candidate gain to a clipped luminance estimate.

**Addresses:** specular highlights smaller than ~67×120 px at 1080p
(one patch on our 16×16 grid) currently slip past the highlight
constraint. A pixel histogram catches them. The iterative estimator
gives the loop a self-damping behaviour near clipping that's stronger
than the per-frame ratio clamp — `ratioMean` based on a single
measurement linearly extrapolates a non-linear plant near saturation,
so dim scenes with bright patches can still overshoot in theory; the
8-pass inner loop converges where the open-loop estimate doesn't.

**Effort:** medium for the histogram side. NEON code changes in
`NeonStatsEncoder.cpp` to maintain bin counts during the existing
single-pass reduce — should fold into the same loop body, ~50 LOC of
NEON intrinsics. IpaStats grows a 128-int field (~512 B). BasicIpa
swaps `top2pcMaxPostWbMean` for a histogram-quantile helper in libipa
shape (~50 LOC). The iterative estimator is independent and small
(~30 LOC) but only worth doing once the histogram is in.

**Caveats:** today's patch-grid based highlight constraint covers
the cases that actually drive AE on this hardware (windows, lamps,
faces — meaningfully sized regions). The pixel-histogram refinement
is a polish for specular highlights that aren't a current real-world
complaint on Mi Pad 1. Defer until either: (a) a real-world scene
shows specular blow-out the patch path missed, or (b) the noise model
(item 10) lands and per-pixel reasoning becomes available for the
denoiser, at which point the histogram cost is amortised.

## Not worth borrowing

A handful of libcamera patterns are **deliberately out of scope** for
this HAL:

- **Mojo IPC + sandboxed IPA process.** Justified for ChromeOS where
  vendor 3A blobs run alongside open code. Our 3A is in-tree and open;
  an IPC boundary is pure overhead. Take the `IpaInterface` shape
  (item 6); skip the IPC.
- **`PipelineHandler` as a base class.** Pays off when there are 8
  pipeline handlers in the tree. We have one. The abstraction would be
  a single subclass with a header full of pure virtuals — premature
  factoring. Revisit if a second SoC lands.
- **YAML tuning format.** Our JSON loader works, schema v1 is shipped,
  the converter (`tools/isp_to_json.py`) targets JSON. Switching format
  costs more than it returns.
- **Software-ISP fallback** (`src/libcamera/software_isp/`). Our Vulkan
  path is the production path; CPU debayer would be a regression on
  Mi Pad 1's CPU. The existing CPU fallback was deleted in Tier 1.5
  for good reason.
- **`awb_nn.cpp`** (RPi NN-AWB). The model dependency is heavier than
  the IQ delta on our class of sensor; the Bayesian variant (item 4)
  captures the win.
- **`media_pipeline.cpp` graph traversal** beyond what item 7 covers.
  RPi-level pipeline reconfiguration on the fly is over-spec for our
  single fixed graph.

## Suggested order of attack

If picking these up incrementally:

1. Item 5 (Fence) — small, decouples a concrete cleanup from larger work.
2. Item 3 (sensor-properties DB) — small, removes a friction point
   before any further sensor work.
3. Item 2 (per-frame `IspParams`) — Tier 3 enabler; everything image-
   quality benefits from per-frame ISP params being a real thing.
4. Item 1 (ALSC) — the image-quality win that closes a logged bug.
5. Item 6 (`IpaInterface` widening) — once items 2 + 1 are in, the
   interface widening has natural anchors.
6. Item 4 (Bayesian AWB) — needs the calibration session, schedule
   when the lab time is available.
7. Items 7-10 — defer until there's a concrete trigger (second SoC,
   denoiser work, video-recording IQ pass, HDR mode).
