# ISP pipeline

`IspPipeline` (`isp/IspPipeline.h`) is the abstraction the HAL talks
to. There's one concrete backend, `VulkanIspPipeline`, and it's the
sole data flow in and out of the HAL — no CPU fallback, no GLES /
HW-ISP backends, no packed-YUV path.

## Interface

The interface is shaped around a **produce-once / sample-many**
multi-stream submit. One Bayer slot demosaics into a per-slot RGBA
scratch image; every output buffer in that frame
(`blitToGralloc` / `blitToYuv` / `blitToJpegCpu`) reads from the
same scratch in a single Vulkan submit.

```cpp
class IspPipeline {
public:
    bool init();
    void destroy();
    void prewarm(unsigned width, unsigned height, uint32_t pixFmt);

    // === Produce-once frame recording ===
    //
    // beginFrame acquires a free CB / scratch / fence slot, waits
    // (poll-fenced) on the slot's previous submit, and starts
    // recording. The compute demosaic of the Bayer slot is the
    // first thing recorded. After this returns, blitTo* may be
    // called any number of times — each appends a render pass /
    // dispatch / copy to the same command buffer. endFrame closes
    // and submits the cmd buffer; the returned submit fence fd is
    // what PipelineThread polls.
    bool beginFrame(unsigned srcW, unsigned srcH, uint32_t pixFmt,
                    int srcInputSlot);
    bool blitToGralloc(void *nativeBuffer,
                       unsigned dstW, unsigned dstH,
                       const CropRect &crop,
                       int acquireFence,
                       int *releaseFenceOut);
    bool blitToYuv    (void *nativeBuffer,
                       unsigned dstW, unsigned dstH,
                       const CropRect &crop,
                       int acquireFence,
                       int *releaseFenceOut);
    bool blitToJpegCpu(JpegSnapshot *out);   // host-mapped RGBA for libjpeg
    void invalidateJpegSnapshot(const JpegSnapshot &snap);
    void releaseJpegSnapshot   (const JpegSnapshot &snap);
    bool endFrame    (int *submitFenceOut);

    // === Bayer input ring (DMABUF) ===
    //
    // The input ring is OPAQUE_FD-exportable, V4l2Device QBUFs into
    // it directly. ISP slot N is referenced by srcInputSlot in
    // beginFrame.
    int    inputBufferCount() const;
    size_t inputBufferSize()  const;
    int    exportInputBufferFd(int idx);
    const void *bayerHost(int slot) const;       // for NEON stats
    void        invalidateBayer(int slot);       // CPU read fence

    // === 3A control surface ===
    //
    // Writes land on the next frame's submit (no silicon delay —
    // these run in the demosaic shader, not in V4L2 controls).
    void setWbGains(unsigned r, unsigned g, unsigned b);   // Q8
    void setCcm    (const int16_t *ccm);                   // Q10 row-major 3×3
    void setEnabled(bool en);
    void setAwbLock(bool lock);

    // === Session lifecycle ===
    void waitForPreviousFrame();                 // drain on stopWorkers
    void onSessionClose();
};
```

`createIspPipeline()` in `isp/IspPipeline.cpp` always returns a
`VulkanIspPipeline`. The legacy HW-ISP backend was deleted during
the Tier 1.5 CPU-fallback purge — recoverable from git history if
the closed `libnvisp_v3` path is ever revived.

Input formats: 10-bit Bayer (`V4L2_PIX_FMT_S{RGGB,GRBG,GBRG,BGGR}10`)
and their 8-bit equivalents. Bayer phase is decoded into the params
buffer the demosaic shader reads.

## Slot ring

Vulkan resources live in a depth-2 round-robin ring
(`SLOT_COUNT = 2` in `VulkanIspPipeline.h`):

- `mScratchImg[k]` — per-slot RGBA scratch, `STORAGE | SAMPLED |
  TRANSFER_SRC`. Pitchlinear, device-local.
- `mDescSet[k]` — descriptor set. Binding 0 = input ring slot
  (rebound per-frame), 1 = `mScratchImg[k]` (storage), 2 =
  `mParamBuf[k]`, 3 = `mScratchView[k]` + sampler.
- `mCmdBuf[k]`, `mFence[k]`, `mSlotSyncFd[k]`, `mParamBuf[k]`,
  `mJpegRing[k]` (host-mapped staging for libjpeg).

Why depth 2 specifically: depth 4 was the original choice and
covered comfortable CPU↔GPU overlap, but the four 1920×1080 RGBA
scratch images plus four 1080p JPEG-ring buffers blew the shared
nvmap pool — `cameraserver` ate ~85 MB and the framework's gralloc
allocator couldn't get a 1920×1080 video buffer when MediaRecorder
asked for one. Tegra K1 serialises submits on its single graphics
queue anyway, so depth 2 carries enough CPU↔GPU overlap on the
hot path while keeping the nvmap budget tight enough that gralloc
keeps allocating.

`acquireSlot()` round-robins through `mNextSlot`. If the chosen
slot's previous submit hasn't signalled, `poll(2)` on the slot's
`sync_fd` blocks until it has — in steady state (frame spacing ≥
one frame_period) the wait is a no-op.

## Per-frame mechanics

`VulkanIspPipeline.cpp` per `beginFrame`:

1. **`acquireSlot()`** — pick slot k, wait its previous fence, drop
   any leftover acquire-fence semaphores from the previous submit.

2. **Compute demosaic** — bind `mDescSet[k]` (binding 0 = input
   ring slot N as a storage buffer), record `vkCmdDispatch` of the
   Malvar-He-Cutler demosaic shader. Output: `mScratchImg[k]` in
   `VK_IMAGE_LAYOUT_GENERAL`, packed RGBA8 with WB + CCM + sRGB
   gamma applied.

3. **Pipeline barrier** — `SHADER_WRITE → SHADER_READ |
   TRANSFER_READ` on `mScratchImg[k]` so the per-output blits /
   encodes / copies see the scratch image stable.

`blitToGralloc(nativeBuffer, …)`:

4. **Acquire-fence semaphore** — if the framework provided an
   `acquire_fence` sync_fd, import it as a binary `VkSemaphore`
   with `TEMPORARY` payload (`VK_KHR_external_semaphore_fd`). The
   submit's `pWaitSemaphores` makes the GPU block on the
   producer's fence instead of stalling the recording thread on a
   `poll(acquireFence)`.

5. **Render pass** — full-screen triangle, fragment shader samples
   `mScratchImg[k]` via `sampler2D` and writes to a colour
   attachment bound to a gralloc-backed `VkImage`. Push-constants
   carry the crop rect (in source coords) and source / destination
   extents, so the same shader handles identity preview, zoom,
   and cross-resolution.

   Fragment ROP is the **only** blocklinear-aware write path
   exposed on Tegra Vulkan — compute-store and `vkCmdCopyImage`
   targeting `VK_ANDROID_native_buffer` both produce swizzled
   garbage on this driver.

6. **Per-output release** — `vkQueueSignalReleaseImageANDROID`
   emits a sync_fence fd; the caller plugs it into
   `camera3_stream_buffer.release_fence` and returns without any
   SW lock on the gralloc buffer.

`blitToYuv(nativeBuffer, …)`:

Same shape but the inner work is a `VulkanYuvEncoder::recordDispatch`
— a compute shader (`RgbaToNv12.h`) that samples the scratch and
writes NV12 into a host-mapped output buffer. Per-slot descriptor
sets in `VulkanYuvEncoder` so the dispatch sees the right scratch
view per slot. After fence signal, `BufferProcessor::processYuvOutput`
locks the gralloc YCbCr buffer with `lockYCbCr`, copies / repacks
NV12 → NV12 / I420 / YV12 via libyuv (NV21 returns `NO_INIT`).
BT.601 limited-range is hardcoded in the shader.

`blitToJpegCpu(out)`:

Picks a free `mJpegRing[s]` host-mapped buffer, records
`vkCmdCopyImageToBuffer` from `mScratchImg[k]` into it, returns a
`JpegSnapshot { rgba, width, height, ringSlot }`. The PipelineThread
forwards the snapshot to `JpegWorker`. `JpegWorker` libjpeg-encodes
into the BLOB gralloc with an EXIF Orientation marker derived from
`ANDROID_JPEG_ORIENTATION` (no pixel rotation in HAL), then calls
`releaseJpegSnapshot(snap)` to free the ring slot. Forgetting that
release would exhaust the ring after `SLOT_COUNT` BLOBs.

`endFrame(submitFenceOut)`:

7. Closes the cmd buffer, builds a single `vkQueueSubmit`:
   - `pWaitSemaphores` = imported acquire-fence semaphores from
     all blits in this frame.
   - `pCommandBuffers` = `mCmdBuf[k]` (compute demosaic + barrier
     + per-output blits / encodes / copies).
   - `pSignalSemaphores` = the per-output
     `vkQueueSignalReleaseImageANDROID` ones (recorded earlier).
   - signal fence = `mFence[k]`; export its fd via
     `vkGetFenceFdKHR(SYNC_FD)` → `submitFenceOut`. The export
     implicitly resets the fence so the slot is ready for reuse.

PipelineThread `poll(2)`s `submitFenceOut`; once it signals,
`StatsProcessStage` and `ResultDispatchStage` run.

## Vulkan resource cache

`VulkanGrallocCache` (`isp/vulkan/io/`) caches the (`VkImage`,
`VkImageView`, `VkFramebuffer`) tuple per `native_handle_t *`. The
per-frame cost of a blit is two descriptor rebinds, a push-constant
upload, and the cmd-buffer record — no Vulkan object creation.
`configureStreams` clears the cache (gralloc handles change across
sessions); `closeDevice` does the same.

## Vulkan loader

Vulkan calls go through a `VulkanPfn` dispatch table populated by
`VulkanLoader`. Two loaders:

- `HalHmiVulkanLoader` — the production loader on Android 7.
  `dlopen`'s `libglcore.so` directly and resolves
  `vkGetInstanceProcAddr` / `vkGetDeviceProcAddr` against it,
  bypassing `libvulkan.so`. The bypass is mandatory because
  `libvulkan.so` filters `VK_ANDROID_native_buffer` out of the
  app-level extension list, and we need that extension to wrap
  gralloc handles as `VkImage`s.
- `SystemVulkanLoader` — a stub that always fails. Reserved for a
  hypothetical Android 8+ port where libvulkan stops filtering.

The `VulkanLoader` abstraction is owned by `VulkanIspPipeline` via
`createVulkanLoader()`; consumers downstream of the abstraction
(every Vulkan call in the codebase) dispatch through `mPfn->X(...)`
exclusively, never against linked `vk*` symbols — the link-time
symbols would re-introduce the libvulkan filter.

## Shaders

`isp/vulkan/shaders/`:

- `DemosaicCompute.h` — Malvar-He-Cutler 5×5 demosaic with a 20×20
  cooperative shared-memory Bayer tile, 16×16 workgroup. Optical
  black subtracted per-channel (`active.opticalBlack.manualBias{R,
  GR,GB,B}` from tuning), dynamic-range rescaled by `255/(maxRaw -
  blackLevel)`. WB gains applied, CCM applied, sRGB gamma
  approximated via a host-shipped 256-entry LUT.
- `Blit.h` — vertex shader emits a full-screen triangle, fragment
  shader samples `mScratchImg` with hardware bilinear and applies
  the push-constant crop / scale. Identity path
  (`cropW == outW && cropH == outH`) uses `texelFetch` for zero
  overhead; scaling path uses the sampler's texture cache.
- `RgbaToNv12.h` — RGBA → NV12 compute shader, BT.601 limited-range
  hardcoded. 8×8 workgroup, one invocation per 4×2 Y block.

## Tegra K1 quirks the code is built around

- **Compute-store to a `VK_ANDROID_native_buffer` image is
  swizzled.** Pitchlinear writes from a compute shader produce
  garbage when the destination is gralloc; only fragment ROP gets
  through the tiler. The whole `blitToGralloc` shape (compute into
  device-local pitchlinear scratch + fragment blit) exists because
  of this.
- **`vkGetMemoryFdPropertiesKHR` on external gralloc fds returns
  `VK_ERROR_INITIALIZATION_FAILED` with `typeBits=0`.** Direct
  gralloc-import as a Vulkan `VkImage` is impossible; the
  fragment-blit round trip is mandatory.
- **Shared-memory `atomicAdd` is software (CAS loop).** ~+18 ms on
  demosaic when it was tried; tree reduces are the only viable
  intra-workgroup reduction primitive.
- **Warp-aligned 2D iteration thrashes the ~12 KB L1 tex cache.**
  Flat-stride loop layout that keeps the warp set compact on 2-3
  rows beats geometric layouts even though it pays an int divmod
  per inner iteration.
- **Fat fused stats-into-demosaic shaders lose ~+27 ms.** Stats
  moved to a CPU NEON pass and the GPU stats compute backend was
  deleted.

These constraints are all measured, not predicted — see the
project memory `reference_tegra_k1_compute.md` (private) and the
on-branch `experiment/stats-fusion` for the GPU-stats post-mortem.

## What the abstraction does NOT do

- **No automatic statistics output.** Statistics live on the CPU
  side via `NeonStatsEncoder` over the Bayer slot. The ISP exports
  the slot's host pointer through `bayerHost(slot)` /
  `invalidateBayer(slot)` so the NEON pass can run in parallel
  with the GPU submit; it doesn't share Vulkan resources with the
  ISP itself.
- **No reprocessing input path.** `Request::inputBuffer` is parsed
  by `StreamConfig::normalize` (one INPUT/BIDIRECTIONAL stream
  allowed) but downstream the slot is unwired.
- **No multi-resolution input.** All blits in a frame sample the
  same scratch — implicit constraint that the V4L2 capture
  resolution matches the largest non-BLOB output.

## Adding a new backend

Highly unlikely on this hardware (Tegra K1 has no other realistic
ISP path), but if a future port revives a HW-ISP option:

1. Subclass `IspPipeline`.
2. Implement `init` / `destroy` / `prewarm`.
3. Implement at least the produce-once frame ops
   (`beginFrame`, one or more `blitTo*`, `endFrame`). The DMABUF
   input-ring helpers are mandatory if the V4L2 capture path stays
   in `V4L2_MEMORY_DMABUF` mode (it should).
4. Implement the 3A control surface — `setWbGains`, `setCcm`,
   `setAwbLock`, `setEnabled`. They feed the per-frame param
   buffer the demosaic stage reads.
5. Wire into `createIspPipeline()`.

Keep the class self-contained — no V4L2 knowledge, no Camera3
metadata. The only inputs are the Bayer slot + the per-frame param
state.
