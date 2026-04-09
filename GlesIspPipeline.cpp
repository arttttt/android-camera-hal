#define LOG_TAG "Cam-GlesISP"
#include <utils/Log.h>
#include <cstring>
#include <linux/videodev2.h>
#include <math.h>
#include <time.h>

static inline int64_t nowMs() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000LL + ts.tv_nsec / 1000000;
}

#include <system/window.h>
#include "GlesIspPipeline.h"

/* EGL/GL extension function pointers */
static PFNEGLCREATEIMAGEKHRPROC eglCreateImageKHR_fn = NULL;
static PFNEGLDESTROYIMAGEKHRPROC eglDestroyImageKHR_fn = NULL;
static PFNGLEGLIMAGETARGETTEXTURE2DOESPROC glEGLImageTargetTexture2DOES_fn = NULL;

static void loadExtFunctions() {
    if (!eglCreateImageKHR_fn) {
        eglCreateImageKHR_fn = (PFNEGLCREATEIMAGEKHRPROC)eglGetProcAddress("eglCreateImageKHR");
        eglDestroyImageKHR_fn = (PFNEGLDESTROYIMAGEKHRPROC)eglGetProcAddress("eglDestroyImageKHR");
        glEGLImageTargetTexture2DOES_fn = (PFNGLEGLIMAGETARGETTEXTURE2DOESPROC)eglGetProcAddress("glEGLImageTargetTexture2DOES");
    }
}

namespace android {

uint8_t GlesIspPipeline::sGammaLut[256];
bool GlesIspPipeline::sGammaReady = false;

void GlesIspPipeline::initGamma() {
    if (sGammaReady) return;
    for (int i = 0; i < 256; i++) {
        float lin = i / 255.0f;
        float s = (lin <= 0.0031308f) ?
            lin * 12.92f :
            1.055f * powf(lin, 1.0f / 2.4f) - 0.055f;
        int v = (int)(s * 255.0f + 0.5f);
        sGammaLut[i] = v > 255 ? 255 : (v < 0 ? 0 : (uint8_t)v);
    }
    sGammaReady = true;
}

static const char *kComputeShaderSrc = R"(#version 310 es
layout(local_size_x = 8, local_size_y = 8) in;

layout(binding = 0) uniform highp usampler2D inTex;
layout(rgba8, binding = 1) writeonly uniform highp image2D outImg;
layout(std430, binding = 2) buffer Params {
    uint width; uint height; uint bayerPhase; uint is16bit;
    uint wbR; uint wbG; uint wbB; uint doIsp;
    int ccm[9];
    uint gammaLut[64];
} params;

uint readPixel(uint x, uint y) {
    uint val = texelFetch(inTex, ivec2(x, y), 0).r;
    return (params.is16bit != 0u) ? val >> 2u : val;
}

float srgbGamma(float lin) {
    return (lin <= 0.0031308) ? lin * 12.92 : 1.055 * pow(lin, 1.0/2.4) - 0.055;
}

void main() {
    uint x = gl_GlobalInvocationID.x;
    uint y = gl_GlobalInvocationID.y;
    if (x >= params.width || y >= params.height) return;

    /* McGuire/Malvar-He-Cutler 5x5 demosaic — 13 samples, branch-free */
    int ix = int(x), iy = int(y);
    int iw = int(params.width) - 1, ih = int(params.height) - 1;
#define PX(dx, dy) float(readPixel(uint(clamp(ix+(dx), 0, iw)), uint(clamp(iy+(dy), 0, ih))))
    float pC  = PX(0, 0);
    float pN  = PX(0,-1);  float pS  = PX(0, 1);
    float pW  = PX(-1, 0); float pE  = PX(1, 0);
    float pN2 = PX(0,-2);  float pS2 = PX(0, 2);
    float pW2 = PX(-2, 0); float pE2 = PX(2, 0);
    float pNW = PX(-1,-1); float pNE = PX(1,-1);
    float pSW = PX(-1, 1); float pSE = PX(1, 1);
#undef PX
    float vFar  = pN2 + pS2;
    float vNear = pN + pS;
    float diag  = pNW + pNE + pSW + pSE;
    float hFar  = pW2 + pE2;
    float hNear = pW + pE;

    float Pcross = (4.0*pC - vFar + 2.0*vNear - hFar + 2.0*hNear) * 0.125;
    float Pcheck = (6.0*pC - 1.5*vFar + 2.0*diag - 1.5*hFar) * 0.125;
    float Ptheta = (5.0*pC + 0.5*vFar - diag - hFar + 4.0*hNear) * 0.125;
    float Pphi   = (5.0*pC - vFar - diag + 0.5*hFar + 4.0*vNear) * 0.125;

    /* Branch-free Bayer position selection */
    uint rX = params.bayerPhase & 1u;
    uint rY = (params.bayerPhase >> 1) & 1u;
    bool isRedRow = ((y + rY) & 1u) == 0u;
    bool isRedCol = ((x + rX) & 1u) == 0u;
    float fR = isRedRow ? (isRedCol ? pC : Ptheta) : (isRedCol ? Pphi : Pcheck);
    float fG = (isRedRow == isRedCol) ? Pcross : pC;
    float fB = isRedRow ? (isRedCol ? Pcheck : Pphi) : (isRedCol ? Ptheta : pC);
    int R = clamp(int(fR + 0.5), 0, 255);
    int G = clamp(int(fG + 0.5), 0, 255);
    int B = clamp(int(fB + 0.5), 0, 255);

    if (params.doIsp != 0u) {
        R = clamp((R * int(params.wbR)) >> 8, 0, 255);
        G = clamp((G * int(params.wbG)) >> 8, 0, 255);
        B = clamp((B * int(params.wbB)) >> 8, 0, 255);
        int rr = clamp((params.ccm[0]*R + params.ccm[1]*G + params.ccm[2]*B) >> 10, 0, 255);
        int gg = clamp((params.ccm[3]*R + params.ccm[4]*G + params.ccm[5]*B) >> 10, 0, 255);
        int bb = clamp((params.ccm[6]*R + params.ccm[7]*G + params.ccm[8]*B) >> 10, 0, 255);
        R = clamp(int(srgbGamma(float(rr)/255.0) * 255.0 + 0.5), 0, 255);
        G = clamp(int(srgbGamma(float(gg)/255.0) * 255.0 + 0.5), 0, 255);
        B = clamp(int(srgbGamma(float(bb)/255.0) * 255.0 + 0.5), 0, 255);
    }

    imageStore(outImg, ivec2(x, y), vec4(float(R)/255.0, float(G)/255.0, float(B)/255.0, 1.0));
}
)";

GlesIspPipeline::GlesIspPipeline()
    : mReady(false), mBufWidth(0), mBufHeight(0)
    , mDisplay(EGL_NO_DISPLAY), mContext(EGL_NO_CONTEXT), mSurface(EGL_NO_SURFACE)
    , mProgram(0), mInTex(0), mParamSSBO(0), mOutTex(0), mFbo(0)
    , mGrallocTex(0), mGrallocFbo(0), mGrallocImage(EGL_NO_IMAGE_KHR), mGrallocHandle(NULL)
    , mInSize(0), mIs16bit(false) {}

GlesIspPipeline::~GlesIspPipeline() { destroy(); }

bool GlesIspPipeline::init() {
    if (mReady) return true;

    /* EGL offscreen context */
    mDisplay = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    if (mDisplay == EGL_NO_DISPLAY || !eglInitialize(mDisplay, NULL, NULL)) {
        ALOGE("eglGetDisplay/Initialize failed");
        return false;
    }

    EGLint cfgAttrs[] = { EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT, EGL_NONE };
    EGLConfig cfg;
    EGLint numCfg;
    eglChooseConfig(mDisplay, cfgAttrs, &cfg, 1, &numCfg);

    /* PBuffer surface for offscreen compute */
    EGLint pbufAttrs[] = { EGL_WIDTH, 1, EGL_HEIGHT, 1, EGL_NONE };
    mSurface = eglCreatePbufferSurface(mDisplay, cfg, pbufAttrs);

    EGLint ctxAttrs[] = { EGL_CONTEXT_CLIENT_VERSION, 3, EGL_NONE };
    mContext = eglCreateContext(mDisplay, cfg, EGL_NO_CONTEXT, ctxAttrs);
    if (mContext == EGL_NO_CONTEXT) {
        ALOGE("eglCreateContext failed: 0x%x", eglGetError());
        destroy();
        return false;
    }

    if (!eglMakeCurrent(mDisplay, mSurface, mSurface, mContext)) {
        ALOGE("eglMakeCurrent failed: 0x%x", eglGetError());
        destroy();
        return false;
    }

    /* Log EGL/GL extensions for dmabuf import capability check */
    const char *eglExts = eglQueryString(mDisplay, EGL_EXTENSIONS);
    if (eglExts) {
        ALOGD("EGL extensions: %s", eglExts);
        if (strstr(eglExts, "EGL_EXT_image_dma_buf_import"))
            ALOGD("EGL_EXT_image_dma_buf_import: AVAILABLE");
    }
    const char *glExts = (const char *)glGetString(GL_EXTENSIONS);
    if (glExts && strstr(glExts, "GL_OES_EGL_image"))
        ALOGD("GL_OES_EGL_image: AVAILABLE");

    /* Compile compute shader — needs current context */
    GLuint shader = glCreateShader(GL_COMPUTE_SHADER);
    glShaderSource(shader, 1, &kComputeShaderSrc, NULL);
    glCompileShader(shader);

    GLint ok;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[512];
        glGetShaderInfoLog(shader, sizeof(log), NULL, log);
        ALOGE("Compute shader compile error: %s", log);
        glDeleteShader(shader);
        destroy();
        return false;
    }

    mProgram = glCreateProgram();
    glAttachShader(mProgram, shader);
    glLinkProgram(mProgram);
    glDeleteShader(shader);

    glGetProgramiv(mProgram, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[512];
        glGetProgramInfoLog(mProgram, sizeof(log), NULL, log);
        ALOGE("Program link error: %s", log);
        destroy();
        return false;
    }

    /* Params SSBO (fixed size) */
    glGenBuffers(1, &mParamSSBO);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, mParamSSBO);
    glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(IspParams), NULL, GL_STREAM_DRAW);

    initGamma();

    /* Release context from init thread — process() will bind on capture thread */
    eglMakeCurrent(mDisplay, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);

    mReady = true;
    ALOGD("GLES compute ISP initialized");
    return true;
}

bool GlesIspPipeline::ensureInput(unsigned width, unsigned height, bool is16) {
    size_t inSize = width * height * (is16 ? 2 : 1);

    if (mInSize >= inSize && mBufWidth == width && mBufHeight == height && mIs16bit == is16)
        return true;

    if (mInTex) glDeleteTextures(1, &mInTex);
    if (mOutTex) glDeleteTextures(1, &mOutTex);
    if (mFbo) glDeleteFramebuffers(1, &mFbo);

    glGenTextures(1, &mInTex);
    glBindTexture(GL_TEXTURE_2D, mInTex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    if (is16)
        glTexStorage2D(GL_TEXTURE_2D, 1, GL_R16UI, width, height);
    else
        glTexStorage2D(GL_TEXTURE_2D, 1, GL_R8UI, width, height);

    glGenTextures(1, &mOutTex);
    glBindTexture(GL_TEXTURE_2D, mOutTex);
    glTexStorage2D(GL_TEXTURE_2D, 1, GL_RGBA8, width, height);

    glGenFramebuffers(1, &mFbo);
    glBindFramebuffer(GL_FRAMEBUFFER, mFbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, mOutTex, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    mInSize = inSize; mIs16bit = is16;
    mBufWidth = width; mBufHeight = height;
    return true;
}

void GlesIspPipeline::dispatchCompute(const uint8_t *src, unsigned width, unsigned height,
                                       bool is16, uint32_t pixFmt) {
    /* Upload input */
    glBindTexture(GL_TEXTURE_2D, mInTex);
    if (is16)
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height,
                        GL_RED_INTEGER, GL_UNSIGNED_SHORT, src);
    else
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height,
                        GL_RED_INTEGER, GL_UNSIGNED_BYTE, src);

    /* Fill params */
    IspParams params = {};
    params.width = width; params.height = height;
    params.is16bit = is16 ? 1 : 0;
    params.doIsp = mEnabled ? 1 : 0;
    params.wbR = mWbR; params.wbG = mWbG; params.wbB = mWbB;

    switch (pixFmt) {
        case V4L2_PIX_FMT_SRGGB10: case V4L2_PIX_FMT_SRGGB8: params.bayerPhase = 0; break;
        case V4L2_PIX_FMT_SGRBG10: case V4L2_PIX_FMT_SGRBG8: params.bayerPhase = 1; break;
        case V4L2_PIX_FMT_SGBRG10: case V4L2_PIX_FMT_SGBRG8: params.bayerPhase = 2; break;
        case V4L2_PIX_FMT_SBGGR10: case V4L2_PIX_FMT_SBGGR8: params.bayerPhase = 3; break;
        default: params.bayerPhase = 0; break;
    }

    if (mCcm)
        for (int i = 0; i < 9; i++) params.ccm[i] = mCcm[i];
    else {
        params.ccm[0] = 1024; params.ccm[4] = 1024; params.ccm[8] = 1024;
    }

    for (int i = 0; i < 64; i++) {
        params.gammaLut[i] = sGammaLut[i*4] |
                             (sGammaLut[i*4+1] << 8) |
                             (sGammaLut[i*4+2] << 16) |
                             (sGammaLut[i*4+3] << 24);
    }

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, mParamSSBO);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, sizeof(IspParams), &params);

    /* Dispatch */
    glUseProgram(mProgram);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, mInTex);
    glBindImageTexture(1, mOutTex, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA8);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, mParamSSBO);
    glDispatchCompute((width + 7) / 8, (height + 7) / 8, 1);
    glMemoryBarrier(GL_FRAMEBUFFER_BARRIER_BIT | GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
}

bool GlesIspPipeline::process(const uint8_t *src, uint8_t *dst,
                               unsigned width, unsigned height,
                               uint32_t pixFmt) {
    if (!mReady) return false;

    if (eglGetCurrentContext() != mContext)
        eglMakeCurrent(mDisplay, mSurface, mSurface, mContext);

    bool is16 = (pixFmt == V4L2_PIX_FMT_SRGGB10 || pixFmt == V4L2_PIX_FMT_SGRBG10 ||
                 pixFmt == V4L2_PIX_FMT_SGBRG10 || pixFmt == V4L2_PIX_FMT_SBGGR10);

    if (!ensureInput(width, height, is16))
        return false;

    int64_t t0 = nowMs();
    dispatchCompute(src, width, height, is16, pixFmt);
    int64_t t1 = nowMs();

    /* Read back via FBO + glReadPixels */
    glBindFramebuffer(GL_READ_FRAMEBUFFER, mFbo);
    glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, dst);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
    int64_t t2 = nowMs();

    ALOGD("GLES: compute=%lld readback=%lldms total=%lld", t1 - t0, t2 - t1, t2 - t0);

    /* AWB from raw input */
    if (mEnabled) {
        uint64_t sR = 0, sG = 0, sB = 0, nR = 0, nG = 0, nB = 0;
        unsigned rX = (pixFmt == V4L2_PIX_FMT_SGRBG10 || pixFmt == V4L2_PIX_FMT_SGRBG8 ||
                       pixFmt == V4L2_PIX_FMT_SBGGR10 || pixFmt == V4L2_PIX_FMT_SBGGR8) ? 1 : 0;
        unsigned rY = (pixFmt == V4L2_PIX_FMT_SGBRG10 || pixFmt == V4L2_PIX_FMT_SGBRG8 ||
                       pixFmt == V4L2_PIX_FMT_SBGGR10 || pixFmt == V4L2_PIX_FMT_SBGGR8) ? 1 : 0;

        for (unsigned y = 0; y < height; y += 7) {
            for (unsigned x = 0; x < width; x += 7) {
                unsigned val;
                if (is16)
                    val = ((const uint16_t *)src)[y * width + x] >> 2;
                else
                    val = src[y * width + x];

                unsigned px = x & 1, py = y & 1;
                if (py == rY && px == rX) { sR += val; nR++; }
                else if (py != rY && px != rX) { sB += val; nB++; }
                else { sG += val; nG++; }
            }
        }

        if (nR && nG && nB) {
            uint64_t avgR = sR / nR, avgG = sG / nG, avgB = sB / nB;
            uint64_t avg = (avgR + avgG + avgB) / 3;
            unsigned r = avgR ? (unsigned)((avg * 256ULL) / avgR) : 256;
            unsigned g = avgG ? (unsigned)((avg * 256ULL) / avgG) : 256;
            unsigned b = avgB ? (unsigned)((avg * 256ULL) / avgB) : 256;
            if (r < 128) r = 128; if (r > 1024) r = 1024;
            if (g < 128) g = 128; if (g > 1024) g = 1024;
            if (b < 128) b = 128; if (b > 1024) b = 1024;
            mWbR = r; mWbG = g; mWbB = b;
        }
    }

    return true;
}

bool GlesIspPipeline::processToGralloc(const uint8_t *src, void *nativeBuffer,
                                        unsigned srcW, unsigned srcH,
                                        unsigned dstW, unsigned dstH,
                                        uint32_t pixFmt) {
    if (!mReady || !nativeBuffer) return false;
    loadExtFunctions();
    if (!eglCreateImageKHR_fn || !glEGLImageTargetTexture2DOES_fn) return false;

    if (eglGetCurrentContext() != mContext)
        eglMakeCurrent(mDisplay, mSurface, mSurface, mContext);

    bool is16 = (pixFmt == V4L2_PIX_FMT_SRGGB10 || pixFmt == V4L2_PIX_FMT_SGRBG10 ||
                 pixFmt == V4L2_PIX_FMT_SGBRG10 || pixFmt == V4L2_PIX_FMT_SBGGR10);

    if (!ensureInput(srcW, srcH, is16))
        return false;

    /* Import gralloc buffer as EGLImage → texture → FBO
     * Cache by ANativeWindowBuffer->handle — buffer pool rotates 3-4 handles */
    ANativeWindowBuffer *anb = (ANativeWindowBuffer *)nativeBuffer;
    void *cacheKey = (void *)anb->handle;
    if (mGrallocHandle != cacheKey) {
        if (mGrallocImage != EGL_NO_IMAGE_KHR)
            eglDestroyImageKHR_fn(mDisplay, mGrallocImage);
        if (mGrallocFbo) { glDeleteFramebuffers(1, &mGrallocFbo); mGrallocFbo = 0; }
        if (mGrallocTex) { glDeleteTextures(1, &mGrallocTex); mGrallocTex = 0; }

        EGLint attrs[] = { EGL_IMAGE_PRESERVED_KHR, EGL_TRUE, EGL_NONE };
        mGrallocImage = eglCreateImageKHR_fn(mDisplay, EGL_NO_CONTEXT,
            EGL_NATIVE_BUFFER_ANDROID,
            (EGLClientBuffer)nativeBuffer, attrs);
        if (mGrallocImage == EGL_NO_IMAGE_KHR) {
            ALOGE("eglCreateImageKHR failed: 0x%x", eglGetError());
            mGrallocHandle = NULL;
            return false;
        }

        glGenTextures(1, &mGrallocTex);
        glBindTexture(GL_TEXTURE_2D, mGrallocTex);
        glEGLImageTargetTexture2DOES_fn(GL_TEXTURE_2D, mGrallocImage);

        glGenFramebuffers(1, &mGrallocFbo);
        glBindFramebuffer(GL_FRAMEBUFFER, mGrallocFbo);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, mGrallocTex, 0);

        GLenum s = glCheckFramebufferStatus(GL_FRAMEBUFFER);
        if (s != GL_FRAMEBUFFER_COMPLETE) {
            ALOGE("Gralloc FBO incomplete: 0x%x", s);
            glBindFramebuffer(GL_FRAMEBUFFER, 0);
            mGrallocHandle = NULL;
            return false;
        }
        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        mGrallocHandle = cacheKey;
        ALOGD("Gralloc buffer imported as EGLImage OK (%ux%u)", dstW, dstH);
    }

    int64_t t0 = nowMs();
    dispatchCompute(src, srcW, srcH, is16, pixFmt);
    int64_t t1 = nowMs();

    /* Blit from mOutTex FBO → gralloc FBO (GPU-internal, no CPU readback) */
    glBindFramebuffer(GL_READ_FRAMEBUFFER, mFbo);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, mGrallocFbo);
    glBlitFramebuffer(0, 0, srcW, srcH, 0, 0, dstW, dstH,
                      GL_COLOR_BUFFER_BIT, GL_NEAREST);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    glFlush(); /* submit commands — gralloc SW_READ lock in Camera.cpp syncs GPU */
    int64_t t2 = nowMs();

    ALOGD("GLES gralloc: compute=%lld blit=%lldms total=%lld", t1 - t0, t2 - t1, t2 - t0);

    /* AWB from raw input */
    if (mEnabled) {
        uint64_t sR = 0, sG = 0, sB = 0, nR = 0, nG = 0, nB = 0;
        unsigned rX = (pixFmt == V4L2_PIX_FMT_SGRBG10 || pixFmt == V4L2_PIX_FMT_SGRBG8 ||
                       pixFmt == V4L2_PIX_FMT_SBGGR10 || pixFmt == V4L2_PIX_FMT_SBGGR8) ? 1 : 0;
        unsigned rY = (pixFmt == V4L2_PIX_FMT_SGBRG10 || pixFmt == V4L2_PIX_FMT_SGBRG8 ||
                       pixFmt == V4L2_PIX_FMT_SBGGR10 || pixFmt == V4L2_PIX_FMT_SBGGR8) ? 1 : 0;
        for (unsigned y = 0; y < srcH; y += 7) {
            for (unsigned x = 0; x < srcW; x += 7) {
                unsigned val = is16 ? ((const uint16_t *)src)[y * srcW + x] >> 2
                                    : src[y * srcW + x];
                unsigned px = x & 1, py = y & 1;
                if (py == rY && px == rX) { sR += val; nR++; }
                else if (py != rY && px != rX) { sB += val; nB++; }
                else { sG += val; nG++; }
            }
        }
        if (nR && nG && nB) {
            uint64_t avgR = sR / nR, avgG = sG / nG, avgB = sB / nB;
            uint64_t avg = (avgR + avgG + avgB) / 3;
            unsigned r = avgR ? (unsigned)((avg * 256ULL) / avgR) : 256;
            unsigned g = avgG ? (unsigned)((avg * 256ULL) / avgG) : 256;
            unsigned b = avgB ? (unsigned)((avg * 256ULL) / avgB) : 256;
            if (r < 128) r = 128; if (r > 1024) r = 1024;
            if (g < 128) g = 128; if (g > 1024) g = 1024;
            if (b < 128) b = 128; if (b > 1024) b = 1024;
            mWbR = r; mWbG = g; mWbB = b;
        }
    }

    return true;
}

void GlesIspPipeline::destroy() {
    if (!mReady && mDisplay == EGL_NO_DISPLAY)
        return;

    if (mDisplay != EGL_NO_DISPLAY) {
        if (mContext != EGL_NO_CONTEXT && mSurface != EGL_NO_SURFACE)
            eglMakeCurrent(mDisplay, mSurface, mSurface, mContext);
        if (mGrallocFbo) { glDeleteFramebuffers(1, &mGrallocFbo); mGrallocFbo = 0; }
        if (mGrallocTex) { glDeleteTextures(1, &mGrallocTex); mGrallocTex = 0; }
        if (mGrallocImage != EGL_NO_IMAGE_KHR && eglDestroyImageKHR_fn)
            { eglDestroyImageKHR_fn(mDisplay, mGrallocImage); mGrallocImage = EGL_NO_IMAGE_KHR; }
        mGrallocHandle = NULL;
        if (mInTex) { glDeleteTextures(1, &mInTex); mInTex = 0; }
        if (mParamSSBO) { glDeleteBuffers(1, &mParamSSBO); mParamSSBO = 0; }
        if (mFbo) { glDeleteFramebuffers(1, &mFbo); mFbo = 0; }
        if (mOutTex) { glDeleteTextures(1, &mOutTex); mOutTex = 0; }
        if (mProgram) { glDeleteProgram(mProgram); mProgram = 0; }
        if (mContext != EGL_NO_CONTEXT) {
            eglMakeCurrent(mDisplay, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
            eglDestroyContext(mDisplay, mContext); mContext = EGL_NO_CONTEXT;
        }
        if (mSurface != EGL_NO_SURFACE) {
            eglDestroySurface(mDisplay, mSurface); mSurface = EGL_NO_SURFACE;
        }
        eglTerminate(mDisplay); mDisplay = EGL_NO_DISPLAY;
    }
    mReady = false;
    mInSize = 0;
    mBufWidth = 0; mBufHeight = 0;
}

}; /* namespace android */
