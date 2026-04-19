# ISP pipeline abstraction

`IspPipeline` (`isp/IspPipeline.h`) is the narrow interface the Camera3
dispatcher talks to. There is one concrete backend —
`VulkanIspPipeline` — and the Bayer-only Vulkan path is the sole data
flow in and out of the HAL.

## Interface

```cpp
class IspPipeline {
public:
    virtual bool init() = 0;
    virtual void destroy() = 0;

    // Zero-copy RGBA: GPU demosaic + optional crop/scale straight into
    // the gralloc buffer. Returns a sync_fence fd for the framework to
    // wait on before compositing. Failure = hardware / driver error;
    // there is no CPU fallback.
    virtual bool processToGralloc(const uint8_t *src, void *nativeBuffer,
                                   unsigned srcW, unsigned srcH,
                                   unsigned dstW, unsigned dstH,
                                   uint32_t pixFmt,
                                   int acquireFence, int *releaseFence,
                                   int srcInputSlot,
                                   const CropRect &crop);

    // Synchronous demosaic into a CPU-readable internal VkBuffer.
    // Returned pointer is valid until the next process*() call. Used
    // by the JPEG path so libjpeg reads RGBA without an extra memcpy.
    virtual const uint8_t *processToCpu(const uint8_t *src,
                                         unsigned width, unsigned height,
                                         uint32_t pixFmt,
                                         int srcInputSlot);

    // Warm the shaders / descriptor bindings at the target stream size
    // so the first real frame isn't JIT-stalled.
    virtual void prewarm(unsigned width, unsigned height, uint32_t pixFmt);

    // DMABUF input ring: capture code exports OPAQUE_FD dma-buf fds of
    // the backend's input buffers and hands them to V4L2 via
    // setDmaBufFds(). processToGralloc / processToCpu reference slot N
    // of this ring via srcInputSlot.
    virtual int    inputBufferCount() const;
    virtual size_t inputBufferSize()  const;
    virtual int    exportInputBufferFd(int idx);

    // Block until async GPU work from the previous processToGralloc
    // has completed. Called before V4L2 reuses a DMABUF slot.
    virtual void   waitForPreviousFrame();

    // 3A helpers — Camera-owned, applied on the next frame.
    void setWbGains(unsigned r, unsigned g, unsigned b);
    void setCcm(const int16_t *ccm);
    void setEnabled(bool en);
    void setAwbLock(bool lock);
};
```

Input formats: 10-bit Bayer (`V4L2_PIX_FMT_SRGGB10` / `SGRBG10` /
`SGBRG10` / `SBGGR10`) and their 8-bit equivalents. Packed YUV is not
supported — the Mi Pad 1 sensors (IMX179 + OV5693) are Bayer only.

## VulkanIspPipeline — per-frame flow

`isp/vulkan/VulkanIspPipeline.cpp`. Per `processToGralloc` call:

1. **Input:** V4L2 writes Bayer straight into slot `i` of the Vulkan
   input ring (owned by `VulkanInputRing`; 4 slots,
   `VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT_KHR`-exportable
   `VkBuffer`s handed to V4L2 via `setDmaBufFds`). No CPU memcpy.
2. **Compute dispatch** reads the slot's `VkBuffer` as an SSBO, runs
   Malvar-He-Cutler demosaic + WB + CCM + sRGB gamma, writes packed
   RGBA8 via `imageStore` into `mScratchImg` (pitchlinear `STORAGE |
   SAMPLED` image, device-local).
3. **Graphics pipeline** — full-screen triangle, fragment shader
   samples `mScratchImg` via `sampler2D` with hardware bilinear and
   outputs to a colour attachment bound to a gralloc-backed `VkImage`.
   A push-constant carries the crop rect (in source-image coords) and
   the source/destination extents, so the same shader handles identity
   preview, zoom, and cross-resolution. This is the only blocklinear-
   aware write path exposed on Tegra Vulkan — compute-store and
   `vkCmdCopyImage` targeting `VK_ANDROID_native_buffer` both produce
   swizzled garbage.
4. **Release:** `vkQueueSignalReleaseImageANDROID` emits a sync_fence
   fd; `Camera::processCaptureRequest` plugs it into
   `camera3_stream_buffer.release_fence` and returns without any SW
   lock on the gralloc buffer.

The gralloc `VkImage` + `VkImageView` + `VkFramebuffer` are cached per
`native_handle_t *` inside `VulkanGrallocCache`, so the per-frame cost
is just two descriptor rebinds, a push-constant upload, and a command
buffer record / submit.

`processToCpu` reuses the same compute dispatch but instead of going
through the graphics blit, it copies `mScratchImg` into a persistent
CPU-mapped `VkBuffer` (`mOutBuf`) via `vkCmdCopyImageToBuffer`, waits on
the fence, and returns the mapped pointer. libjpeg reads from it
directly — no intermediate scratch on the host.

Vulkan functions are loaded through `VulkanLoader` + `VulkanPfn`
dispatch table instead of linking `libvulkan` directly. The HAL-variant
`HalHmiVulkanLoader` bypasses Android 7's libvulkan filter and exposes
`VK_ANDROID_native_buffer`, which is unavailable to app-side consumers.

## Backend selection

`IspPipeline *createIspPipeline()` in `isp/IspPipeline.cpp` always
returns a `VulkanIspPipeline`. The HW-ISP backend was deleted during
the Tier 1.5 CPU-fallback purge — recoverable from git history if
the NV `libnvisp_v3` path ever needs reviving.

## What this abstraction does NOT do today

- **No statistics output.** AF reads the rendered preview via a
  one-per-frame `SW_READ_OFTEN` lock on a gralloc output during sweep.
  AE / AWB have no feedback at all. Surfacing statistics would be the
  single biggest quality upgrade — see Tier 3's IPA module in
  [roadmap.md](roadmap.md).

- **No per-frame tuning-parameter channel.** `process*` take only
  `width`, `height`, `pixFmt`, and the crop rect. CCM / gamma / WB are
  set once via `setCcm` / `setWbGains`. A useful evolution: an
  `IspParameters` struct populated by a 3A module and passed per-frame.

- **No CPU/GPU frame overlap beyond pipeline-depth 1.**
  `VulkanIspPipeline` submits async and returns immediately, but a
  single `mCmdBuf` / `mFence` forces the next frame to wait on
  `waitForPreviousFrame()` before recording. Ping-pong command buffers
  + the request-queue refactor in Tier 3 would unlock it.

- **No producer-once sampled-many.** Each `processToGralloc` call runs
  its own compute demosaic; on a multi-output frame the same Bayer is
  demosaiced N times. Tier 3.5 changes this — see
  [roadmap.md](roadmap.md).

## Adding a new backend

1. Subclass `IspPipeline`.
2. Implement `init` (allocate context, shaders, buffers) and `destroy`
   (release).
3. Implement at least `processToGralloc` and `processToCpu`. The
   DMABUF-input methods are optional — default impls return 0 and the
   capture code falls back to MMAP + memcpy from `src`.
4. Wire into `createIspPipeline()` and `Android.mk`.

Keep the class self-contained — no direct V4L2 knowledge, no Camera3
metadata. The only inputs are `(src-or-srcInputSlot, width, height,
pixFmt, crop)`.
