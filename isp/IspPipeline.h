#ifndef ISP_PIPELINE_H
#define ISP_PIPELINE_H

#include <stdint.h>

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
     * For single-shot capture (JPEG). Default calls process(). */
    virtual bool processSync(const uint8_t *src, uint8_t *dst,
                              unsigned width, unsigned height,
                              uint32_t pixFmt) {
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
     * Returns false if not supported — caller falls back to process(). */
    virtual bool processToGralloc(const uint8_t *src, void *nativeBuffer,
                                   unsigned width, unsigned height,
                                   uint32_t pixFmt,
                                   int acquireFence, int *releaseFence) {
        (void)src; (void)nativeBuffer; (void)width; (void)height; (void)pixFmt;
        (void)acquireFence;
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
