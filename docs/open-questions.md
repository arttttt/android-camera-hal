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
