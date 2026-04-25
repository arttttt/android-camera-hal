#ifndef ISP_PIPELINE_H
#define ISP_PIPELINE_H

#include <stdint.h>
#include <stddef.h>

#include "CropRect.h"
#include "JpegSnapshot.h"

namespace android {

/* Common ISP interface — CPU and GPU implementations */
class IspPipeline {
public:
    virtual ~IspPipeline() {}

    virtual bool init() = 0;
    virtual void destroy() = 0;

    /* Synchronous demosaic into an internal CPU-readable RGBA buffer
     * owned by the backend. Returns a pointer to tightly-packed RGBA8
     * (row stride = width * 4), valid until the next process*() call
     * on this pipeline. Returns NULL on failure. Used by the JPEG path
     * where libjpeg reads the RGBA directly. */
    virtual const uint8_t *processToCpu(const uint8_t *src,
                                         unsigned width, unsigned height,
                                         uint32_t pixFmt,
                                         int srcInputSlot) {
        (void)src; (void)width; (void)height; (void)pixFmt; (void)srcInputSlot;
        return NULL;
    }

    /* Run a dummy dispatch at the target stream dimensions so the backend's
     * shader / GPU state is warm before the first real frame. Default no-op. */
    virtual void prewarm(unsigned width, unsigned height, uint32_t pixFmt) {
        (void)width; (void)height; (void)pixFmt;
    }

    /* Produce-once / sample-many API.
     *
     * Demosaic happens once per frame; the resulting RGBA scratch is then
     * sampled / copied N times (one per output stream in the request).
     *
     * Lifecycle:
     *   beginFrame(...)            -> records demosaic into scratch
     *   blitToGralloc(...)         -> records sampler-blit into RGBA gralloc
     *   blitToYuv(...)             -> records compute NV12 encode into NV12 gralloc
     *   endFrame()                 -> submits the recorded cmd buffer; release
     *                                 fences populated by blitTo* are valid
     *                                 once endFrame returns true.
     *
     * Single submit per frame: every blitTo* shares the same VkSubmit, so all
     * outputs of one frame complete on the GPU at the same time. Per-output
     * release fences are dup()'d copies of the same signal. Acquire fences are
     * imported as binary VkSemaphores so the submit waits on them GPU-side;
     * the recording thread never blocks on framework backpressure.
     *
     * On error inside any blitTo*, the frame is aborted: recording state is
     * reset, no submit is issued, and the caller must not invoke endFrame.
     * Default base implementations return false / set releaseFence=-1, leaving
     * production-impl-required behaviour at the override point. */

    virtual bool beginFrame(unsigned srcW, unsigned srcH, uint32_t pixFmt,
                             int srcInputSlot) {
        (void)srcW; (void)srcH; (void)pixFmt; (void)srcInputSlot;
        return false;
    }

    /* Record a sampler-blit of the scratch image into the gralloc nativeBuffer
     * (RGBA8 stream). Identity dst==src uses texelFetch in the shader; otherwise
     * hardware bilinear via sampler2D. acquireFence ownership transfers to the
     * impl on success (closed/imported). */
    virtual bool blitToGralloc(void *nativeBuffer,
                                unsigned dstW, unsigned dstH,
                                const CropRect &crop,
                                int acquireFence,
                                int *releaseFenceOut) {
        (void)nativeBuffer; (void)dstW; (void)dstH; (void)crop;
        (void)acquireFence;
        *releaseFenceOut = -1;
        return false;
    }

    /* Record a compute NV12 encode of the scratch image into the gralloc
     * nativeBuffer (YCbCr_420_888 stream). Width must be a multiple of 4 and
     * height a multiple of 2. acquireFence semantics match blitToGralloc. */
    virtual bool blitToYuv(void *nativeBuffer,
                            unsigned dstW, unsigned dstH,
                            const CropRect &crop,
                            int acquireFence,
                            int *releaseFenceOut) {
        (void)nativeBuffer; (void)dstW; (void)dstH; (void)crop;
        (void)acquireFence;
        *releaseFenceOut = -1;
        return false;
    }

    /* Record a vkCmdCopyImageToBuffer of the scratch image into a backend-
     * owned host-mapped RGBA8 buffer, picked from a small ring so multiple
     * BLOB requests can be in flight without overwriting each other.
     * On success populates *out with the buffer's CPU pointer + dimensions
     * + an opaque ringSlot handle. The pointer is valid for CPU read after
     * endFrame's submit fence has signalled and invalidateJpegSnapshot has
     * been called. Caller must invoke releaseJpegSnapshot once the data
     * is no longer needed so the ring slot rotates back. */
    virtual bool blitToJpegCpu(JpegSnapshot *out) {
        if (out) {
            out->rgba     = nullptr;
            out->width    = 0;
            out->height   = 0;
            out->size     = 0;
            out->ringSlot = -1;
        }
        return false;
    }

    /* Invalidate the CPU cache for the snapshot's buffer so a host read
     * after the GPU write sees coherent data. Called by the consumer
     * right before libjpeg reads. */
    virtual void invalidateJpegSnapshot(const JpegSnapshot &snap) { (void)snap; }

    /* Mark the snapshot's ring slot free for reuse by future blitToJpegCpu
     * calls. Must be called exactly once per successful blitToJpegCpu. */
    virtual void releaseJpegSnapshot(const JpegSnapshot &snap) { (void)snap; }

    /* Host-mapped NV12 buffer the most recent blitToYuv encoded into. Valid
     * only after endFrame's fence has been reaped and invalidateYuvForCpu()
     * has been called. The buffer is shared across the (at most) one
     * in-flight frame; multi-frame pipelining would need a per-slot ring. */
    virtual const uint8_t *yuvHostBuffer() const { return nullptr; }

    /* Invalidate the CPU cache for the YUV host buffer so a host read after
     * the GPU write sees coherent data. Called by the consumer right before
     * libyuv reads the buffer. */
    virtual void invalidateYuvForCpu() {}

    /* Submit the recorded command buffer. Must be called once per matching
     * beginFrame, after all blitTo* calls.
     * submitFenceOut: [out, may be NULL] sync_fd that signals when the
     *                 submit completes. Caller-owned; push into
     *                 ctx.pendingFenceFds for the consumer's poll set.
     *                 Equivalent (timing-wise) to any per-output release
     *                 fence; provided as a separate handle so frames with
     *                 only non-fence outputs (YUV / BLOB) still expose a
     *                 completion signal. -1 if the backend cannot export.
     * Returns false on submit failure; release fences from blitTo* are
     * invalid in that case. */
    virtual bool endFrame(int *submitFenceOut) {
        if (submitFenceOut) *submitFenceOut = -1;
        return false;
    }

    /* AWB gains (Q8: 256 = 1.0x) */
    void setWbGains(unsigned r, unsigned g, unsigned b) {
        mWbR = r; mWbG = g; mWbB = b;
    }

    /* CCM matrix (Q10: 1024 = 1.0x), 3x3 row-major */
    void setCcm(const int16_t *ccm) { mCcm = ccm; }

    /* Optical-black bias, in the sensor's native range (10-bit or 8-bit
     * depending on pixel format). Applied per pixel before demosaic so
     * shadows hit true 0 and full 255 dynamic range is preserved. */
    void setBlackLevel(uint32_t v) { mBlackLevel = v; }

    /* Enable/disable ISP processing (false = demosaic only) */
    void setEnabled(bool en) { mEnabled = en; }

    /* Lock AWB — freeze gains during AF sweep */
    void setAwbLock(bool lock) { mAwbLocked = lock; }
    bool awbLocked() const     { return mAwbLocked; }

    /* Number of exportable Bayer input buffers the backend has pre-allocated.
     * 0 means the backend does not support DMABUF input. */
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

    /* Called by Camera::closeDevice after workers have been stopped
     * and GPU drained. The backend should release any per-session
     * references it is holding — notably cached bindings into
     * framework-owned gralloc buffers, which become invalid the
     * moment the session ends. Core Vulkan resources (device,
     * scratch image, input ring, shaders) stay since the pipeline
     * instance survives close → reopen. */
    virtual void onSessionClose() {}

    /* Read-only host pointer to the raw Bayer content of input ring
     * slot `slot`. The pointer is valid for the lifetime of the
     * backend, but the content is only defined while V4L2 holds the
     * slot dequeued (i.e. between BayerSource::acquireNextFrame and
     * the matching releaseFrame) and after invalidateBayer(slot) has
     * been called. Intended for CPU stats / analysis on the raw
     * capture. Returns NULL on backends without a CPU-mapped input
     * ring. */
    virtual const void *bayerHost(int slot) const { (void)slot; return nullptr; }

    /* Invalidate the CPU cache range backing bayerHost(slot) so a
     * host read after the V4L2 write (and any subsequent GPU read)
     * sees coherent data. No-op on backends that serve bayerHost
     * from HOST_COHERENT memory, and on those that don't expose one
     * at all. */
    virtual void invalidateBayer(int slot) { (void)slot; }

protected:
    unsigned mWbR = 256, mWbG = 256, mWbB = 256;
    const int16_t *mCcm = nullptr;
    uint32_t mBlackLevel = 0;
    bool mEnabled = true;
    bool mAwbLocked = false;
};

/* Pick and construct an ISP backend based on runtime properties.
 * Caller owns the returned instance and must call init() before use. */
IspPipeline *createIspPipeline();

}; /* namespace android */

#endif // ISP_PIPELINE_H
