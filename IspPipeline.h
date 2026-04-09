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

    /* Zero-copy: process from dmabuf fd instead of CPU pointer.
     * Default falls back to regular process(). */
    virtual bool processFromDmabuf(int dmabufFd, const uint8_t *cpuFallback,
                                    uint8_t *dst, unsigned width, unsigned height,
                                    uint32_t pixFmt) {
        return process(cpuFallback, dst, width, height, pixFmt);
    }

    /* AWB gains (Q8: 256 = 1.0x) */
    void setWbGains(unsigned r, unsigned g, unsigned b) {
        mWbR = r; mWbG = g; mWbB = b;
    }

    /* CCM matrix (Q10: 1024 = 1.0x), 3x3 row-major */
    void setCcm(const int16_t *ccm) { mCcm = ccm; }

    /* Enable/disable ISP processing (false = demosaic only) */
    void setEnabled(bool en) { mEnabled = en; }

protected:
    unsigned mWbR = 256, mWbG = 256, mWbB = 256;
    const int16_t *mCcm = nullptr;
    bool mEnabled = true;
};

}; /* namespace android */

#endif // ISP_PIPELINE_H
