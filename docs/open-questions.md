# Open questions — to investigate

Questions whose answer would change a design choice but aren't blocking
current work. Each entry: what's unknown, why it matters, where the
answer might live.

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

## YUV_420_888 — "proper" GPU-direct and NV21 support

**Current state (Tier 2):** `IspPipeline::processToYuv420` runs a
compute shader that writes NV12 into a host-mapped `VkBuffer`;
`BufferProcessor::processYuvOutput` `lockYCbCr`s the gralloc target
and repacks via libyuv (`CopyPlane` for NV12 target, `NV12ToI420` for
I420 / YV12). NV21 is rejected with `NO_INIT`. BT.601 limited-range
is hardcoded in the shader. Extra CPU memory traffic: one full Y copy
(~2 MB at 1080p) + one UV copy/split (~1 MB) per frame ≈ 3-5 ms on
Tegra LPDDR3.

**Question 1: can we skip the CPU repack entirely?**
Ideal path: the compute shader writes **directly** into the gralloc
buffer's YUV planes. That needs either:

- A Vulkan YCbCr image format exposed to storage-image writes (Vulkan
  1.1+ `VK_KHR_sampler_ycbcr_conversion` — not on Tegra K1).
- `vkGetMemoryFdPropertiesKHR` accepting a gralloc fd so we can
  `vkImportMemory` it — returns `VK_ERROR_INITIALIZATION_FAILED`
  on our driver (see `memory/reference_tegra_vulkan.md`).

Neither available, so CPU repack stays. The question reopens if we
ever move to Vulkan 1.1 (not reachable from Tegra K1's NV driver) or
if Tegra gralloc gains a `VK_ANDROID_native_buffer`-style YUV variant
(no public evidence of this).

**Question 2: NV21 targets.**
The Android-7 libyuv we link has no direct `NV12ToNV21` and scalar
U/V swap is ~20 ms at 1080p (unacceptable on the hot path). Options:

- Chain `NV12ToI420` → `I420ToNV21` via a temp 1 MB plane. Two
  conversions, but both NEON-accelerated — estimated 5-8 ms total
  including alloca'd temp. Acceptable if any consumer actually picks
  NV21.
- Write a dedicated NEON swap using `vld2.8 / vst2.8` (8-byte
  deinterleave + byte-swap). ~2 ms at 1080p. Low maintenance burden
  but ties us to ARM NEON.
- Wait for Tier 3.5 (produce-once / sample-many) to reshape the
  GPU shader to target the layout the consumer asked for directly —
  shader branches on chroma-step and byte-order flag from a push
  constant. Pushes the swap to GPU, frees the CPU entirely, and
  unifies with other per-output shader variants.

Resolving this gets flagged if we see a real consumer on the device
request NV21 (log line `YUV layout not supported (chroma_step=2,
cb=%p, cr=%p)` with `cb > cr`). Until then the stub is OK.

**Question 3: BT.709 + full-range and colour-space metadata.**
Today we bake BT.601 limited into the shader and emit no
`ANDROID_COLOR_CORRECTION_*` / sensor colour-space keys, which is
what every Android-side YUV consumer defaults to. If / when we start
supporting manual colour spaces (required for serious video
pipelines, HDR, etc.) the coefficients move into a push constant and
we fill in the relevant static / per-frame metadata. Not scheduled.

## CCM / WB — deferred CCT-driven selection

**Current state (Tier 2):** `Camera::configureStreams` picks the
nearest-CCT CCM from `SensorTuning::ccmSets()` with a hardcoded
target of 5000 K (daylight). The `wbGain` field in each Set is
ignored — our `VulkanIspPipeline::updateAwb` runs a live gray-world
estimator instead and applies the result directly to the demosaic
shader's `wbR/wbG/wbB` parameters.

This is visibly wrong in non-daylight scenes: under tungsten (~2800 K)
we apply a daylight CCM to pixels that are already warm, skin goes
orange-yellow. Under cool white fluorescent (~4100 K) everything
drifts slightly green.

**Question: when does proper CCT-driven selection land?**

A complete implementation has three pieces, all blocked on the Tier 3
AWB module:

1. **Per-frame CCT estimate.** The existing gray-world loop estimates
   a chromaticity but we throw it away rather than classify it into a
   CCT. A second pass converting gray-world RGB gains to an illuminant
   temperature — via the `awb.v4.UtoCCT` / `CCTtoU` tables in the
   tuning's `reserved` section — gives us an integer CCT per frame.

2. **Interpolated CCM selection.** Swap `ccmForCctQ10(5000, …)` for
   `ccmForCctQ10(estimatedCcm, …)` plus linear interpolation between
   the two ccmSets bracketing the estimate. Prevents visible "snaps"
   when the scene CCT crosses a tuning-set boundary.

3. **`wbGain` as AWB prior.** Each ccmSet carries three-channel
   white-balance gains tuned for that CCT. The AWB output should lean
   on these instead of (or in addition to) the live gray-world estimate
   — the stock tuning represents a lot of testing over sample scenes
   that the runtime estimator can't recreate.

`mwbCCT.{cloudy,shade,incandescent,fluorescent}` are for the *manual*
WB presets (`ANDROID_CONTROL_AWB_MODE = INCANDESCENT / FLUORESCENT / …`),
a separate but related consumer — those four points clamp the CCT
target when the user selects a preset. Same plumbing as (2), different
driver.
