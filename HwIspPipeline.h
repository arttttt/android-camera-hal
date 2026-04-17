#ifndef HW_ISP_PIPELINE_H
#define HW_ISP_PIPELINE_H

#include "IspPipeline.h"
#include <stdint.h>
#include <stddef.h>

namespace android {

/*
 * Hardware ISP pipeline — uses Tegra T124 ISP-A via MIUI libnvisp_v3.so blob.
 *
 * Flow:
 *   1. init(): dlopen blobs, NvIspOpen, HwSettingsCreate+Apply (calibration)
 *   2. process(): upload raw to nvmap, submit reprocess gather (format 0x43),
 *                 read RGBA output
 *   3. destroy(): close ISP, free nvmap handles
 *
 * Requires:
 *   - libnvos.so, libnvrm.so, libnvrm_graphics.so, libnvisp_v3.so in LD_LIBRARY_PATH
 *   - nvrm_shim.so loaded via LD_PRELOAD (for NVMAP_IOC_MMAP compat)
 *   - /dev/nvhost-isp accessible (chmod 666 or run as camera group)
 */
class HwIspPipeline : public IspPipeline {
public:
    HwIspPipeline();
    ~HwIspPipeline();

    bool init() override;
    void destroy() override;

    bool process(const uint8_t *src, uint8_t *dst,
                 unsigned width, unsigned height,
                 uint32_t pixFmt) override;

private:
    /* nvmap helpers */
    uint32_t nvmapCreate(uint32_t size);
    int nvmapAlloc(uint32_t handle);
    int nvmapWrite(uint32_t handle, uint32_t offset, const void *data, uint32_t size);
    int nvmapRead(uint32_t handle, uint32_t offset, void *data, uint32_t size);
    uint32_t nvmapPin(uint32_t handle);
    void nvmapFree(uint32_t handle);

    /* ISP submit */
    bool submitStrip(int strip, int numStrips,
                     uint32_t inH, uint32_t outH,
                     uint32_t cmdH, uint32_t statsH);

    bool mReady;

    /* fds */
    int mNvmapFd;
    int mIspFd;
    int mCtrlFd;

    /* ISP blob handles */
    void *mIspHandle;
    void *mHwSettings;

    /* blob function pointers */
    void *mLibIsp;
    typedef uint32_t (*NvIspClose_t)(void *);
    typedef uint32_t (*NvIspHwSettingsDestroy_t)(void *);
    NvIspClose_t mPIspClose;
    NvIspHwSettingsDestroy_t mPHwDestroy;

    /* syncpoints */
    uint32_t mSpMemory;

    /* allocated buffers (persistent across frames) */
    uint32_t mInHandle, mOutHandle, mStatsHandle, mCmdHandle;
    uint32_t mInIova, mOutIova, mStatsIova;
    unsigned mBufWidth, mBufHeight;
    size_t mInSize, mOutSize;
};

}; /* namespace android */

#endif // HW_ISP_PIPELINE_H
