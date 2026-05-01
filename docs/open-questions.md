# Open questions — to investigate

Questions whose answer would change a design choice but aren't blocking
current work. Each entry: what's unknown, why it matters, where the
answer might live.

## AE per-frame variability source on IMX179

**Question:** what generates the regular per-frame variability on the
rear camera that the AE close-speed boost (Step 3 of the Tier 3 AE
refactor) resonates with?

Empirical observation: with `active.hal_overrides.ae.close_speed_zone
> 0` on IMX179, AE gain swings frame-to-frame at large amplitude even
when the scene looks visually static. Setting `close_speed_zone = 0`
(single-pole LPF at τ ≈ 5 frames everywhere, no faster pole near
target) makes the swings disappear. OV5693 with `close_speed_zone =
0.2` is stable on the same kind of test, so it's not the boost itself
that's wrong — something on the rear cam path provides input
variability in the band the boost amplifies.

Plausible sources, none confirmed:

- **Mains flicker.** 33 ms exposure ≈ 3.3 cycles of 50 Hz mains;
  different frames sample different parts of the cycle, giving 5-10 %
  brightness flutter per frame. RPi's AGC pins shutter to multiples
  of `flickerPeriod` and trades the residue into gain — we don't.
- **AWB → iqmHi cascade.** `lastWbR/B` drift between frames (visible
  in 3A logs: ~9 % shift in `lastWbB` over 2 s); `iqmHi` is computed
  as max-of-channels post-WB, so the highlight constraint candidate
  moves whenever WB does. On a coloured scene this cascades through
  `min(ratioMean, ratioHighlight)` even when the underlying luma is
  steady. OV5693 on a dim test scene sits below the AWB confidence
  gate and stays on priors — no AWB flutter, no cascade.
- **Lens-shading absent.** IMX179 has 20-30 % corner falloff; ROI
  patches near the vignette boundary swing in mean-luma with sub-
  pixel camera shake.
- **V4L2 gain quantization.** Old `bugs.md` hypothesis. Kernel driver
  may round Q8 gain coarser than we ask for — AE writes 1430, sensor
  applies 1408 every other frame, AE chases its own delayed
  observation.
- **Real scene variability.** Faces, breathing, hands, shadows.

**Why it matters:** if the source is identified and addressed at the
source (anti-flicker / AWB damping / LSC / kernel patch), the boost
in Step 3 could be re-enabled on IMX179 for faster final-approach
convergence. Until then `close_speed_zone = 0` mitigates the symptom
without addressing cause.

**Where to look:**

- Per-frame (not per-32-frame) `Cam-BasicIpa: 3A:` log over a held
  static scene, FFT the gain trajectory — period of resonance points
  at the frequency of the variability source.
- Compare manual-AE preview at fixed exposure: if brightness still
  flutters → mains flicker or sensor noise; if steady → it's the AE
  loop with WB cascade or quantization.
- Disable AWB (force priors) on rear cam, re-test boost — isolates
  AWB cascade as a candidate.

## HW JPEG encoder on Tegra K1 / Android 7

**Question:** is any Tegra K1 hardware JPEG encoder block reachable
from our Android 7 HAL?

Current assumption (unverified): NVENC / NVJPEG are closed blobs, no
public Vulkan / V4L2 / MediaCodec surface exposes them, so `libjpeg`
on CPU is the only option. ~150 ms per 8 MP shot is unavoidable with
this assumption.

**Why it matters:** if there *is* a reachable HW JPEG path, the whole
Tier 1.5 step 3 changes shape — demosaiced pixels go straight to the
encoder, no CPU readback, JPEG encode drops from 150 ms to likely
<20 ms.

**Where to look:**

- `/dev/nvhost-*` nodes on the device — do any advertise a JPEG
  capability? `nvhost-mpe` / `nvhost-msenc` / `nvhost-vic` are
  suspicious names.
- libnvomx / libnvmm blobs in `/system/lib` — OMX encoder for
  `image/jpeg`? MediaCodec enumeration (`MediaCodecList`) may expose
  it from userspace.
- Tegra X1 downstream kernel sources (public) for `nvhost-nvjpg` —
  the K1 kernel is similar enough that the same ioctl / node layout
  likely applies if the block is in silicon.
- L4T documentation for Jetson TK1 — if a HW JPEG is exposed there,
  the same path should work from Android.

If found, the implementation would likely sit in a new
`isp/hw/JpegEncoderHw.{h,cpp}` behind an `IspPipeline`-level interface
so the CPU libjpeg path stays as the fallback.

## YUV_420_888 — GPU-direct write and NV21 support

**Current state:** `VulkanIspPipeline::blitToYuv` runs a compute
shader (`RgbaToNv12.h`) that samples the per-slot scratch image and
writes NV12 into a host-mapped `VkBuffer`;
`BufferProcessor::processYuvOutput` then `lockYCbCr`s the gralloc
target and repacks via libyuv (`CopyPlane` for NV12 target,
`NV12ToI420` for I420 / YV12). NV21 returns `NO_INIT`. BT.601
limited-range is hardcoded in the shader. Extra CPU memory traffic:
one full Y copy (~2 MB at 1080p) + one UV copy / split (~1 MB) per
frame ≈ 3-5 ms on Tegra LPDDR3.

**Question 1: can we skip the CPU repack entirely?**
Ideal path: the compute shader writes **directly** into the gralloc
buffer's YUV planes. That needs either:

- A Vulkan YCbCr image format exposed to storage-image writes
  (Vulkan 1.1+ `VK_KHR_sampler_ycbcr_conversion` — not on Tegra K1).
- `vkGetMemoryFdPropertiesKHR` accepting a gralloc fd so we can
  `vkImportMemory` it — returns `VK_ERROR_INITIALIZATION_FAILED`
  on this driver.

Neither available, so the CPU repack stays. The question reopens
only if Tegra K1's NV Vulkan driver gains a YUV-target equivalent
of `VK_ANDROID_native_buffer` (no public evidence of this).

**Question 2: NV21 targets.**
Android-7's libyuv has no direct `NV12 → NV21` and a scalar U/V
swap is ~20 ms at 1080p — unacceptable on the hot path. Options:

- Chain `NV12 → I420 → NV21` via a temp 1 MB plane. Two conversions,
  both NEON-accelerated; estimated 5-8 ms.
- Dedicated NEON swap using `vld2.8 / vst2.8`; ~2 ms at 1080p.
- Branch the `RgbaToNv12` shader on a chroma-step / byte-order
  push constant so the GPU writes the consumer's layout directly.
  Frees the CPU entirely, no extra plane allocation; cost is one
  shader variant.

Resolving this triggers if a real consumer on the device requests
NV21 (log line `YUV layout not supported (chroma_step=2, cb=%p,
cr=%p)` with `cb > cr`). Until then the stub is fine.

**Question 3: BT.709 + full-range and colour-space metadata.**
Today we bake BT.601 limited into the shader and emit no
`ANDROID_COLOR_CORRECTION_*` / sensor colour-space keys, which is
what every Android-side YUV consumer defaults to. If / when we start
supporting manual colour spaces (required for serious video
pipelines, HDR, etc.) the coefficients move into a push constant and
we fill in the relevant static / per-frame metadata. Not scheduled.

## HAL binary still in `/system/lib/hw/`, not `/vendor/lib/hw/`

All other Tegra HALs on Mi Pad 1 (`gralloc.tegra.so`,
`hwcomposer.tegra.so`, `vulkan.tegra.so`) live under
`/vendor/lib/hw/`. Our `camera.tegra.so` is the odd one out —
`Android.mk`'s `BUILD_SHARED_LIBRARY` emits to `/system/lib/hw/` by
default, and we haven't flipped it.

**Why it matters:** every tuning JSON the HAL reads already lives
under `/vendor/etc/camera/tuning/` (Tier 2). The .so should follow
for consistency and so a future real-vendor-partition Treble port
is a single build flag rather than a surgery across install paths.

**What to change:**
- Add `LOCAL_PROPRIETARY_MODULE := true` (or set `LOCAL_VENDOR_MODULE`
  per current AOSP naming) to the shared-library section in
  `Android.mk`.
- The init / cameraserver already looks in `/vendor/lib/hw/`
  alongside `/system/lib/hw/`, so no manifest edit.
- Deploy path in `reference_build_workflow.md` updates from
  `/system/lib/hw/camera.tegra.so` to `/vendor/lib/hw/camera.tegra.so`.

**Why not now:** verified that everything works in `/system/lib/hw/`
on the current ROM. Moving the install path mid-feature-work is a
context-switch tax; bundling it with the eventual "true Treble
port" (split vendor partition, vendor HIDL HAL) makes more sense.


## Manual AWB presets (INCANDESCENT / FLUORESCENT / …)

CCT-driven CCM selection is wired (Tier 3 PR 6.7): `BasicIpa`
estimates a CCT from the gray-world G/B ratio via the tuning's
`awb.v4.UtoCCT` polynomial, picks / interpolates a CCM between the
calibrated `colorCorrection.Set[].cct` brackets, applies the
matching `wbGain` priors as the AWB neutral anchor, and falls back
to the prior on the confidence gate. So the bulk of the historical
AWB question is closed.

What's still **not** wired: the `mwbCCT.{cloudy, shade,
incandescent, fluorescent}` preset reference points. Camera2's
`ANDROID_CONTROL_AWB_MODE` accepts those four (plus DAYLIGHT,
TWILIGHT, WARM_FLUORESCENT) as user-selectable presets that clamp
the CCT target instead of running gray-world. We currently treat
every non-AUTO / non-OFF AWB mode as "hold last gains" — manual
presets work as a freeze, but they don't actually clamp to a
per-preset CCT.

**Question: is preset support worth wiring?**

The same plumbing as the auto path: pick the matching `mwbCCT`
entry's CCT, drive the existing CCM LERP / wbGain prior path with
it, ignore the gray-world estimate while the preset is active.
`AWB_MODE_AUTO` already runs gray-world; `AWB_MODE_OFF` already
honours `COLOR_CORRECTION_GAINS`. Adding the seven preset modes
is a bounded extension of the AWB switch.

Worth doing if a real consumer asks for it; not blocking common
camera apps (which use AUTO).
