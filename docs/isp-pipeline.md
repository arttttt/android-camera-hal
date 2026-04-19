# ISP pipeline abstraction

The `IspPipeline` interface (`isp/IspPipeline.h`) hides which backend
performs demosaic / colour correction / format conversion. Two
implementations are live today: `VulkanIspPipeline` (the default
soft-ISP) and `HwIspPipeline` (the Tegra hardware ISP via the
`libnvisp_v3` blob — kept for reference, only partially working).

## Interface

```cpp
class IspPipeline {
public:
    virtual bool init() = 0;
    virtual void destroy() = 0;

    // Synchronous RGBA readback into dst. CPU-fallback path for JPEG
    // encode or when zero-copy output is ineligible.
    virtual bool process(const uint8_t *src, uint8_t *dst,
                         unsigned width, unsigned height,
                         uint32_t pixFmt) = 0;

    virtual bool processSync(const uint8_t *src, uint8_t *dst,
                             unsigned width, unsigned height,
                             uint32_t pixFmt,
                             int srcInputSlot = -1);

    // Warm shader / command-buffer state at the target stream size so
    // the first real frame isn't JIT-stalled.
    virtual void prewarm(unsigned width, unsigned height, uint32_t pixFmt);

    // Zero-copy RGBA output: GPU writes straight into the gralloc
    // buffer; release fence is emitted through *releaseFence for the
    // framework to wait on before compositing. Returns false when the
    // backend can't do zero-copy for this call (caller falls back to
    // process()).
    virtual bool processToGralloc(const uint8_t *src, void *nativeBuffer,
                                  unsigned width, unsigned height,
                                  uint32_t pixFmt,
                                  int acquireFence, int *releaseFence,
                                  int srcInputSlot = -1);

    // DMABUF input ring: backends that can consume V4L2 capture
    // directly into device-visible memory advertise a non-zero count
    // here; the capture code then exports fds and hands them to V4L2
    // via setDmaBufFds(). process*(srcInputSlot=N) references slot N
    // of this ring instead of memcpy-ing from src.
    virtual int    inputBufferCount() const;
    virtual size_t inputBufferSize()  const;
    virtual int    exportInputBufferFd(int idx);

    // Block until async GPU work from the previous process*() call
    // has completed. Called before V4L2 reuses a DMABUF slot.
    virtual void   waitForPreviousFrame();

    // 3A helpers.
    void setWbGains(unsigned r, unsigned g, unsigned b);
    void setCcm(const int16_t *ccm);
    void setEnabled(bool en);
    void setAwbLock(bool lock);
};
```

`src` layout depends on `pixFmt`:

- `V4L2_PIX_FMT_UYVY`, `V4L2_PIX_FMT_YUYV` — packed YUV. Never hits
  `IspPipeline` on this SoC; `ImageConverter` handles them directly.
- `V4L2_PIX_FMT_SRGGB10` / `SGRBG10` / `SGBRG10` / `SBGGR10` — 10-bit
  Bayer packed in 16-bit containers. Backends do full debayer + white
  balance + colour correction + gamma.

## Backends

### `VulkanIspPipeline`

`isp/vulkan/VulkanIspPipeline.cpp`. The production soft-ISP. Pipeline per frame:

1. **Input:** V4L2 writes Bayer straight into slot `i` of the Vulkan
   input ring (owned by `VulkanInputRing`; 4 slots,
   `VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT_KHR`-exportable
   `VkBuffer`s handed to V4L2 via `setDmaBufFds`). No CPU memcpy.
2. **Compute dispatch** reads the slot's `VkBuffer` as an SSBO, runs Malvar-He-Cutler
   demosaic + WB + CCM + sRGB gamma, writes packed RGBA8 via
   `imageStore` into `mScratchImg` (pitchlinear `STORAGE` image,
   device-local).
3. **Graphics pipeline** — full-screen triangle, fragment shader
   `imageLoad`s from `mScratchImg` and outputs to a colour attachment
   bound to a gralloc-backed `VkImage`. This is the only blocklinear-
   aware write path exposed on Tegra Vulkan — compute-store and
   `vkCmdCopyImage` targeting `VK_ANDROID_native_buffer` both produce
   swizzled garbage.
4. **Release:** `vkQueueSignalReleaseImageANDROID` emits a sync_fence
   fd; `Camera::processCaptureRequest` plugs it into
   `camera3_stream_buffer.release_fence` and returns without any
   SW lock on the gralloc buffer.

The gralloc `VkImage` + `VkImageView` + `VkFramebuffer` are cached per
`native_handle_t *` inside `VulkanGrallocCache`, so the per-frame
cost is just two descriptor rebinds and a command buffer record /
submit.

Vulkan functions are loaded through `VulkanLoader` +
`VulkanPfn` dispatch table instead of linking `libvulkan` directly.
The HAL-variant `HalHmiVulkanLoader` bypasses Android 7's libvulkan
filter and exposes `VK_ANDROID_native_buffer`, which is unavailable to
app-side consumers.

### `HwIspPipeline`

`isp/hw/HwIspPipeline.cpp`. Uses Tegra ISP-A through the MIUI
`libnvisp_v3.so` blob. Flow:

1. `dlopen` the blob stack (`libnvos`, `libnvrm`, `libnvrm_graphics`,
   `libnvisp_v3`). Requires `nvrm_shim.so` via `LD_PRELOAD` for
   `NVMAP_IOC_MMAP` compat on newer kernels.
2. `NvIspOpen`, `HwSettingsCreate`, `HwSettingsApply` — applies the
   blob's baked calibration.
3. Per frame: allocate nvmap handles for input / output / stats / cmd,
   upload raw, submit an ISP "reprocess gather" with format `0x43`,
   wait on syncpoint, read output RGBA.
4. `NvIspClose`, `HwSettingsDestroy`, free nvmap on teardown.

Selected when `persist.camera.soft_isp=0`. Partially works — most of
the time the blob matches the kernel well enough to produce frames,
but output is sometimes corrupt and there's no path to owning the
tuning. Kept around as a reference for what a hardware ISP path looks
like from userspace, not as a production target.

## Backend selection

`IspPipeline *createIspPipeline()` in `isp/IspPipeline.cpp`, driven by
`persist.camera.soft_isp` (default `1`):

```cpp
char val[PROPERTY_VALUE_MAX] = {0};
property_get("persist.camera.soft_isp", val, "1");
return (val[0] != '0') ? (IspPipeline *) new VulkanIspPipeline()
                       : (IspPipeline *) new HwIspPipeline();
```

No runtime fallback: if the chosen backend fails `init()`,
`Camera::configureStreams` returns `NO_INIT` and the HAL proceeds with
no ISP. A `Hw → Vulkan` fallback chain would be a cheap resilience
win — tracked in [roadmap.md](roadmap.md).

## What this abstraction does NOT do today

- **No statistics output.** AF reads the rendered preview. AE / AWB
  have no feedback at all. `HwIspPipeline` allocates a stats nvmap
  handle (`mStatsHandle`, `isp/hw/HwIspPipeline.h:72`) but nothing reads
  it. Surfacing statistics would be the single biggest quality
  upgrade.

- **No per-frame tuning-parameter channel.** `process*` take only
  `width, height, pixFmt`. CCM / gamma / WB are set once via `setCcm`
  / `setWbGains`. A useful evolution: an `IspParameters` struct
  populated by a 3A module and passed per-frame.

- **No shared temp buffer pool.** `mRgbaTemp` is a single buffer per
  `Camera`; if we ever pipeline multiple in-flight requests, we need
  a small pool.

- **No CPU/GPU frame overlap.** `VulkanIspPipeline` submits async and
  returns immediately, but a single `mCmdBuf` / `mFence` forces the
  next frame to wait on `waitForPreviousFrame()` before recording.
  Ping-pong command buffers + the request-queue refactor in
  [roadmap.md](roadmap.md) (Tier 3) would double throughput.

## Adding a new backend

1. Subclass `IspPipeline`.
2. Implement `init` (allocate context, shaders, buffers) and
   `destroy` (release).
3. Implement at least `process` (synchronous CPU readback fallback).
   `processToGralloc` and the DMABUF-input methods are optional —
   default impls return `false` / `0` and the camera stays on the
   CPU path.
4. Wire into `createIspPipeline()` and `Android.mk`.

Keep the class self-contained — no direct V4L2 knowledge, no Camera3
metadata. The only input is `(src-or-srcInputSlot, width, height,
pixFmt)`.
