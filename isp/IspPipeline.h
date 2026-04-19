#ifndef ISP_PIPELINE_H
#define ISP_PIPELINE_H

#include <stdint.h>
#include <stddef.h>

namespace android {

/* Common ISP interface — CPU and GPU implementations */
class IspPipeline {
public:
    virtual ~IspPipeline() {}

    virtual bool init() = 0;
    virtual void destroy() = 0;

    /*
     * Process raw Bayer frame → RGBA8888.
     * src: Bayer input (8-bit or 10-bit 16-bit LE)
     * dst: RGBA output buffer
     * Returns true on success.
     */
    virtual bool process(const uint8_t *src, uint8_t *dst,
                         unsigned width, unsigned height,
                         uint32_t pixFmt) = 0;

    /* Synchronous process — waits for GPU completion, result in dst immediately.
     * For single-shot capture (JPEG). srcInputSlot has the same meaning as in
     * processToGralloc(). Default calls process(). */
    virtual bool processSync(const uint8_t *src, uint8_t *dst,
                              unsigned width, unsigned height,
                              uint32_t pixFmt,
                              int srcInputSlot = -1) {
        (void)srcInputSlot;
        return process(src, dst, width, height, pixFmt);
    }

    /* Run a dummy dispatch at the target stream dimensions so the backend's
     * shader / GPU state is warm before the first real frame. Default no-op. */
    virtual void prewarm(unsigned width, unsigned height, uint32_t pixFmt) {
        (void)width; (void)height; (void)pixFmt;
    }

    /* Process Bayer and blit result directly to gralloc buffer (no CPU readback).
     * nativeBuffer: ANativeWindowBuffer* cast to void*.
     * acquireFence: caller-owned fd that signals when buffer is ready for GPU writes.
     *               Implementation consumes (closes) it on success, leaves open on failure.
     *               Pass -1 if no fence.
     * releaseFence: [out] non-null. Set to sync_fence fd that signals when GPU is done.
     *               Caller owns and must close. -1 means no fence (invalid / fell back).
     * srcInputSlot: if >=0, the Bayer data is already in the backend's input
     *               ring slot `srcInputSlot` (DMABUF capture path); `src` is
     *               ignored. If -1, the backend memcpy's `src` into its own
     *               staging buffer.
     * Returns false if not supported — caller falls back to process(). */
    virtual bool processToGralloc(const uint8_t *src, void *nativeBuffer,
                                   unsigned width, unsigned height,
                                   uint32_t pixFmt,
                                   int acquireFence, int *releaseFence,
                                   int srcInputSlot = -1) {
        (void)src; (void)nativeBuffer; (void)width; (void)height; (void)pixFmt;
        (void)acquireFence; (void)srcInputSlot;
        *releaseFence = -1;
        return false;
    }

    /* AWB gains (Q8: 256 = 1.0x) */
    void setWbGains(unsigned r, unsigned g, unsigned b) {
        mWbR = r; mWbG = g; mWbB = b;
    }

    /* CCM matrix (Q10: 1024 = 1.0x), 3x3 row-major */
    void setCcm(const int16_t *ccm) { mCcm = ccm; }

    /* Enable/disable ISP processing (false = demosaic only) */
    void setEnabled(bool en) { mEnabled = en; }

    /* Lock AWB — freeze gains during AF sweep */
    void setAwbLock(bool lock) { mAwbLocked = lock; }

    /* Number of exportable Bayer input buffers the backend has pre-allocated.
     * 0 means the backend does not support DMABUF input — callers must feed
     * Bayer data through process()/processToGralloc's src pointer instead. */
    virtual int inputBufferCount() const { return 0; }

    /* Size (bytes) of each input buffer. Valid once the backend has been
     * configured for the current resolution (after prewarm()). */
    virtual size_t inputBufferSize() const { return 0; }

    /* Export input buffer `idx` as an OPAQUE_FD dma-buf fd. Returns -1 on
     * failure. Caller owns the fd and must close it when done; the backend
     * keeps its own reference to the underlying VkDeviceMemory. */
    virtual int exportInputBufferFd(int idx) { (void)idx; return -1; }

    /* Block until any async GPU work submitted by the previous process*
     * call has completed. Safe to call unconditionally. Needed before
     * returning an input buffer to V4L2 on the DMABUF capture path so VI
     * doesn't overwrite a slot the shader is still reading. */
    virtual void waitForPreviousFrame() {}

protected:
    unsigned mWbR = 256, mWbG = 256, mWbB = 256;
    const int16_t *mCcm = nullptr;
    bool mEnabled = true;
    bool mAwbLocked = false;
};

/* Pick and construct an ISP backend based on runtime properties.
 * Caller owns the returned instance and must call init() before use. */
IspPipeline *createIspPipeline();

}; /* namespace android */

#endif // ISP_PIPELINE_H
