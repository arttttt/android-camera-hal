#ifndef CPU_ISP_PIPELINE_H
#define CPU_ISP_PIPELINE_H

#include "IspPipeline.h"
#include "ImageConverter.h"

namespace android {

class CpuIspPipeline : public IspPipeline {
public:
    bool init() override { return true; }
    void destroy() override {}

    bool process(const uint8_t *src, uint8_t *dst,
                 unsigned width, unsigned height,
                 uint32_t pixFmt) override {
        mConverter.BayerToRGBA(src, dst, width, height, pixFmt, mEnabled);
        return true;
    }

private:
    ImageConverter mConverter;
};

}; /* namespace android */

#endif // CPU_ISP_PIPELINE_H
