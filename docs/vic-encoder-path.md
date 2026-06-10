# VIC encoder-stream path (dormant)

Status: **disabled**. `BufferProcessor::processOne` routes YCbCr_420_888
outputs through the GPU NV12 encode + libyuv repack path unconditionally
(`d428626`). The NvBlit deferral machinery — `processYuvNvBlit`, the
`DemosaicBlitStage` third pass, `PipelineContext::outputDeferredNvBlit` —
is still in the tree but unreachable. This document is the evidence
record and the re-enable roadmap.

## Why it exists, and why it is off

The VIC bridge (`5690f2e`) was built on the theory that NVENC ingests
pitch-linear NV12 through a ~5 fps "MSENC mode 1" tiling pre-blit, so
feeding it tiled NV12 written by VIC (the same engine NVENC uses
internally) would unlock 1080p30.

Both halves of that theory died in June 2026:

1. **The 5 fps was never the pre-blit.** The SmokeR24.1 kernel prints
   ~15 `falcon msenc/vic03: pin:` debug lines per engine job; with
   `console=ttyS0,115200n8` at loglevel 7 every encoder job serialized
   behind ~1.3 KB of synchronous 115200-baud UART printk (~190 ms).
   With the console capped (`loglevel=4` in the device cmdline) the
   "slow" GPU+libyuv writer records a measured **29.9 fps** (297
   frames / 9.9 s of audio clock, zero CameraSource drops,
   19–22 ms/frame post). Reference: `screenrecord` — no camera in the
   path — went 11 → 247 frames/10 s on the same loglevel flip.

2. **The VIC output is tile-scrambled in every combination tried.**
   NVENC's metadata-mode input treats external ANW buffers as pitch
   unconditionally (the mode-1 pre-blit always runs), and NvBlit, as
   driven through our ABI, does not honour the surface-kind
   descriptors of the gralloc handles:

   | dst layout | scratch (src) layout | result |
   |---|---|---|
   | tiled NV12 (fmt 34 kept, 12288K bufs) | blocklinear descriptor | tiles |
   | pitch NV12 (fmt 35 + SW_WRITE_OFTEN) | blocklinear descriptor | tiles |
   | pitch NV12 (fmt 35 + SW_WRITE_OFTEN) | pitch descriptor (SW flag on scratch) | tiles |

   Photos (BLOB) and preview read the same scratch through the GPU and
   are correct — only the consumer that trusts the handle descriptors
   (VIC) scrambles, which points at the descriptors/ABI, not at the
   image content.

A correctness footnote that cost real time: `tools/nvblit_probe`
validated a **solid red fill** by average-Y. Uniform content is
layout-invariant — the probe cannot detect tiling/scramble bugs and
proved nothing beyond "the blit engine runs and converts colour".

## Related correctness fixes that stay

- Encoder stream contract: `IMPLEMENTATION_DEFINED` is rewritten to
  YCbCr_420_888 + SW_WRITE_OFTEN → **pitch-linear NV12**
  (`StreamConfig.cpp`). Pitch is what NVENC's input stage reads;
  keeping fmt 34 allocates tiled and encodes as garbage. Note gralloc0
  EINVALs fmt 35 with pure-HW usage (the framework misreports this as
  "Out of memory" — the April "nvmap OOM" was that misread).
- `processYuvNvBlit` reaps the VIC release fence in-HAL instead of
  passing NvBlit's ReleaseSync downstream (CameraSource's 200 ms
  memory-base pool turns per-frame fence waits into dropped frames).
- `finalizeCpuOutputs` skips outputs flagged `outputDeferredNvBlit` —
  the CPU repack used to stomp VIC-written buffers with the stale host
  YUV copy.

## Re-enable roadmap (in order; do not skip steps)

1. **Get the real NvBlit ABI.** The `NvBlitState` struct in
   `isp/nvblit/NvBlitContext.cpp` is hand-declared guesswork; the
   surface fields are the prime suspect (does the blob take
   `buffer_handle_t`, or `NvRmSurface*`?). Two routes:
   - The NvBlit API shipped with public headers in some Tegra K1-era
     L4T R21.x BSP releases — find `nvblit.h` there first.
   - `/vendor/lib/libnvblit.so` is 47 KB — small enough to fully
     disassemble if no headers surface.
2. **Rewrite `nvblit_probe`** with a gradient or checkerboard fill and
   per-pixel dst validation. Then run the full surface matrix
   (pitch→pitch, blocklinear→pitch, pitch→blocklinear,
   blocklinear→blocklinear) and record which combinations the blob
   handles honestly.
3. Only with (1)+(2) green: re-enable the deferral branch in
   `processOne` (single `if` — see `932c43c` for the shape) and verify
   on-device with a **structured scene**, not a dark room.

Expected win for the effort: drop ~10–15 ms/frame of CPU (libyuv) and
the GPU NV12 pass — power/thermals more than fps, since the SW path
already sustains 30 fps. An alternative half-step with the same win:
GPU writes NV12 directly into the pitch gralloc (no VIC), but the K1
Vulkan driver's 2-plane / linear ANW import support needs its own
probe first.

## File map

- `hal/pipeline/BufferProcessor.cpp` — `processOne` YCbCr case (the
  disabled branch point), `processYuvNvBlit` (dormant), fence reap,
  `finalizeCpuOutputs` guard.
- `hal/pipeline/stages/DemosaicBlitStage.cpp` — pass 3 (dormant).
- `hal/pipeline/StreamConfig.cpp` — format rewrite + usage contract.
- `isp/nvblit/NvBlitContext.cpp` — dlopen wrapper, hand-declared ABI.
- `tools/nvblit_probe/` — the probe to rewrite.
- Device tree `BoardConfig.mk` — `loglevel=4` cmdline (the actual
  speed fix).
