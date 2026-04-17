# ISP pipeline abstraction

The `IspPipeline` interface (`IspPipeline.h`) hides which backend
performs demosaic / colour correction / format conversion. Four
implementations exist today.

## Interface

```cpp
class IspPipeline {
public:
    virtual bool init() = 0;
    virtual void destroy() = 0;

    // Blocking CPU path: src → dst, both in system memory.
    virtual bool processSync(const uint8_t *src, uint8_t *dst,
                             unsigned width, unsigned height,
                             uint32_t pixFmt) = 0;

    // dmabuf-in, CPU-mapped-buffer-out. Fast path for GPU backends
    // that can consume dmabuf directly; CPU fallback pointer is
    // provided for backends that cannot.
    virtual bool processFromDmabuf(int dmabufFd, const uint8_t *cpuFallback,
                                   uint8_t *dst,
                                   unsigned width, unsigned height,
                                   uint32_t pixFmt) = 0;

    // dmabuf-in, gralloc-out. Zero-copy end-to-end on GPU backends —
    // writes directly into the compositor-facing RGBA buffer. Not all
    // backends implement it (returns false to signal fallback).
    virtual bool processToGralloc(const uint8_t *src, void *nativeBuffer,
                                  unsigned srcW, unsigned srcH,
                                  unsigned dstW, unsigned dstH,
                                  uint32_t pixFmt);

    // AWB lock toggle used during AF sweeps. Default no-op for backends
    // without an AWB loop.
    virtual void setAwbLock(bool locked) {}
};
```

`src` layout depends on `pixFmt`:

- `V4L2_PIX_FMT_UYVY`, `V4L2_PIX_FMT_YUYV` — packed YUV. Backends skip
  demosaic, just do colour-space conversion to RGBA.
- `V4L2_PIX_FMT_SRGGB10` / `SGRBG10` / `SGBRG10` / `SBGGR10` — 10-bit
  Bayer packed 16-bit. Backends do full debayer + white-balance +
  colour-correction + gamma.

## Backends

### `CpuIspPipeline`

`CpuIspPipeline.cpp`. libyuv-based, runs on `Workers` thread pool for
row parallelism. Slow on Bayer input (naive bilinear demosaic in C++).
Fine on packed YUV. Kept for debug / correctness reference.

### `GlesIspPipeline`

`GlesIspPipeline.cpp`. OpenGL ES 3.0 pipeline, EGL-initialised. Runs as
a fragment shader reading a GL_TEXTURE_2D_RGBA sampled as packed Bayer,
writing into a gralloc-backed EGLImage via FBO when `processToGralloc`
is called. Zero-copy output on the golden path; dmabuf input via
`EGL_LINUX_DMA_BUF_EXT`.

### `VulkanIspPipeline`

`VulkanIspPipeline.cpp`. Compute-shader pipeline. Consumes dmabuf via
`VK_EXT_external_memory_dma_buf`, outputs either to mapped system
memory or to gralloc through `VK_ANDROID_external_memory_android_hardware_buffer`.
Currently the preferred soft-ISP path on this HAL when
`soft_isp=1`: modestly faster than GLES, cleaner code.

### `HwIspPipeline`

`HwIspPipeline.cpp`. Uses Tegra ISP-A via the MIUI `libnvisp_v3.so` blob.
Flow, per `HwIspPipeline.h`:

1. `dlopen` the blob stack (`libnvos`, `libnvrm`, `libnvrm_graphics`,
   `libnvisp_v3`). Requires `nvrm_shim.so` via `LD_PRELOAD` for
   `NVMAP_IOC_MMAP` compat on newer kernels.
2. `NvIspOpen`, `HwSettingsCreate`, `HwSettingsApply` — applies
   calibration baked into the blob.
3. Per frame: allocate nvmap handles for input / output / stats / cmd,
   upload raw, submit an ISP "reprocess gather" with format `0x43`,
   wait on syncpoint, read output RGBA.
4. On teardown, `NvIspClose`, `HwSettingsDestroy`, free nvmap.

Selected when `ro.hal.camera.soft_isp=0`. Fastest and highest quality
path — full hardware ISP with tuned calibration — but fragile: depends
on NVIDIA blobs matching the kernel, and we do not own the tuning.

## Backend selection

At `Camera` construction time (`Camera.cpp` around `mSoftIspEnabled`):

```cpp
mSoftIspEnabled = property_get_bool("ro.hal.camera.soft_isp", true);
if (mSoftIspEnabled) {
    mIsp = new VulkanIspPipeline();      // or GlesIspPipeline as fallback
} else {
    mIsp = new HwIspPipeline();
}
```

There is no runtime fallback if the chosen backend fails `init()` — we
log the error and the HAL proceeds with a broken ISP. Adding a graceful
fallback chain `Hw → Vulkan → GLES → CPU` is a low-effort resilience
win.

## Things this abstraction does not do today

- **No statistics output.** AF reads the rendered preview. AE / AWB
  have no feedback at all. A proper ISP exposes an AE histogram, AWB
  patch averages, and an AF sharpness grid via a separate output
  channel — `HwIspPipeline` actually has a `mStatsHandle` nvmap
  allocation (`HwIspPipeline.h:72`), but nothing reads it. Surfacing
  statistics would be the single biggest quality upgrade.

- **No per-frame tuning parameters.** `processSync` etc. take only
  `width, height, pixFmt`. Exposure / gain / WB gains / CCM matrix /
  gamma are either baked into the blob (`HwIspPipeline`) or hardcoded
  (`VulkanIspPipeline`). A useful evolution: add a
  `IspParameters { awbGainR, awbGainB, ccm[3][3], blackLevel, … }` struct
  passed to every `process*` call, populated by the 3A module.

- **No shared temp buffer pool.** `mRgbaTemp` is a single buffer per
  `Camera`; if we ever pipeline multiple requests we need a small pool.

- **No explicit GPU fence returned.** `processFromDmabuf` waits inside
  the call (`vkQueueWaitIdle` or `glFinish`). Returning a sync fd
  would let the framework thread proceed while the GPU is still
  writing, overlapping with JPEG encode / result marshalling. Covered
  in [roadmap.md](roadmap.md).

## Adding a new backend

1. Subclass `IspPipeline`.
2. Implement `init` (allocate context, shaders, buffers) and
   `destroy` (release).
3. Implement at least `processSync`. `processFromDmabuf` and
   `processToGralloc` are optional — the default
   `processToGralloc` returns `false` which triggers the CPU readback
   path in `Camera.cpp`.
4. Wire into the `Camera` constructor selector and `Android.mk`.

Keep the class self-contained (no direct V4L2 knowledge, no Camera3
metadata) — the only input is `(src, width, height, pixFmt)`.
