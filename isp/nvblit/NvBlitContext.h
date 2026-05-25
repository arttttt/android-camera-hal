#ifndef ISP_NVBLIT_NVBLITCONTEXT_H
#define ISP_NVBLIT_NVBLITCONTEXT_H

#include <stdint.h>

#include <hardware/gralloc.h>

namespace android {

/* RAII wrapper around libnvblit.so — Tegra K1's HW VIC blit engine
 * exposed by NVIDIA blob. We use it to convert demosaiced RGBA
 * (gralloc-backed scratch) into tiled NV12 (gralloc-backed encoder
 * output) in a single HW pass, bypassing the CPU libyuv repack that
 * caps the YCbCr_420_888 encoder path at ~5-7 fps.
 *
 * Loaded via dlopen at init(); init() returns false if the blob is
 * missing or the API isn't usable. Callers should treat NvBlitContext
 * the same way as VulkanIspPipeline — construct, init(), check
 * isReady(); the HAL falls back to the libyuv path if it isn't.
 *
 * Thread safety: per nvblit.h the API is thread-safe, the same context
 * can be used concurrently from multiple threads. */
class NvBlitContext {
public:
    NvBlitContext();
    ~NvBlitContext();

    NvBlitContext(const NvBlitContext&) = delete;
    NvBlitContext& operator=(const NvBlitContext&) = delete;

    /* Open libnvblit and create a VIC context. Idempotent — calling
     * twice is a no-op. */
    bool init();

    bool isReady() const { return mCtx != nullptr; }

    /* Simple integer rectangle for src/dst sub-region selection. The
     * underlying API takes floats for sub-pixel src precision; we
     * convert internally. */
    struct Rect {
        int x, y, w, h;
    };

    /* HW-accelerated blit src → dst with optional rects.
     *
     * srcRect, dstRect: nullptr means full surface. If both are set
     *   with different sizes, VIC scales (with bilinear filtering).
     *
     * srcFence, dstFence: sync_fds the HW waits on before reading src
     *   / writing dst. Pass -1 if no fence. On success the wrapper
     *   takes ownership and the underlying API closes them.
     *
     * releaseFenceOut: sync_fd that signals when the blit has finished
     *   writing dst. Caller takes ownership and must close it. Set to
     *   -1 on failure.
     *
     * Returns true on success. On failure the source and destination
     * fences are NOT consumed — caller still owns them. */
    bool blit(buffer_handle_t src, const Rect *srcRect, int srcFence,
              buffer_handle_t dst, const Rect *dstRect, int dstFence,
              int *releaseFenceOut);

private:
    void *mLib;        /* dlopen handle for libnvblit.so */
    void *mCtx;        /* NvBlitContext* opaque (from NvBlitOpen) */

    /* Function pointers resolved via dlsym at init(). */
    typedef int32_t (*pfn_NvBlitOpen)(void **ctx);
    typedef void    (*pfn_NvBlitClose)(void *ctx);
    typedef int32_t (*pfn_NvBlit)(void *ctx, const void *state, void *result);

    pfn_NvBlitOpen  mOpen;
    pfn_NvBlit      mBlit;
    pfn_NvBlitClose mClose;
};

} /* namespace android */

#endif /* ISP_NVBLIT_NVBLITCONTEXT_H */
