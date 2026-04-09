#ifndef GLES_ISP_PIPELINE_H
#define GLES_ISP_PIPELINE_H

#include "IspPipeline.h"

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES3/gl31.h>
#include <GLES2/gl2ext.h>

namespace android {

class GlesIspPipeline : public IspPipeline {
public:
    GlesIspPipeline();
    ~GlesIspPipeline();

    bool init() override;
    void destroy() override;

    bool process(const uint8_t *src, uint8_t *dst,
                 unsigned width, unsigned height,
                 uint32_t pixFmt) override;

    bool processToGralloc(const uint8_t *src, void *nativeBuffer,
                           unsigned srcW, unsigned srcH,
                           unsigned dstW, unsigned dstH,
                           uint32_t pixFmt) override;

private:
    bool mReady;
    unsigned mBufWidth, mBufHeight;

    EGLDisplay mDisplay;
    EGLContext mContext;
    EGLSurface mSurface;

    GLuint mProgram;
    GLuint mInTex, mParamSSBO;
    GLuint mOutTex, mFbo;
    GLuint mGrallocTex, mGrallocFbo;
    EGLImageKHR mGrallocImage;
    void *mGrallocHandle;  /* last imported ANativeWindowBuffer* */
    size_t mInSize;
    bool mIs16bit;

    bool ensureInput(unsigned width, unsigned height, bool is16);
    void dispatchCompute(const uint8_t *src, unsigned width, unsigned height,
                          bool is16, uint32_t pixFmt);

    static uint8_t sGammaLut[256];
    static bool sGammaReady;
    static void initGamma();

    /* Must match shader layout */
    struct IspParams {
        uint32_t width;
        uint32_t height;
        uint32_t bayerPhase;
        uint32_t is16bit;
        uint32_t wbR, wbG, wbB;
        uint32_t doIsp;
        int32_t ccm[9];
        uint32_t gammaLut[64];
    };
};

}; /* namespace android */

#endif // GLES_ISP_PIPELINE_H
