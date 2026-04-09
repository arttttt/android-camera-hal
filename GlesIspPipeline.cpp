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

#include "GlesIspPipeline.h"

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
layout(local_size_x = 16, local_size_y = 16) in;

layout(std430, binding = 0) buffer InputBuf  { uint data[]; } inBuf;
layout(rgba8, binding = 1) writeonly uniform highp image2D outImg;
layout(std430, binding = 2) buffer Params {
    uint width; uint height; uint bayerPhase; uint is16bit;
    uint wbR; uint wbG; uint wbB; uint doIsp;
    int ccm[9];
    uint gammaLut[64];
} params;

uint readPixel(uint x, uint y) {
    if (params.is16bit != 0u) {
        uint idx = y * params.width + x;
        uint word = inBuf.data[idx >> 1u];
        uint val = ((idx & 1u) == 0u) ? (word & 0xFFFFu) : (word >> 16);
        return val >> 2;
    } else {
        uint idx = y * params.width + x;
        uint word = inBuf.data[idx >> 2u];
        return (word >> ((idx & 3u) * 8u)) & 0xFFu;
    }
}

uint gammaLookup(uint val) {
    if (val > 255u) val = 255u;
    uint wordIdx = val >> 2;
    uint byteIdx = val & 3u;
    return (params.gammaLut[wordIdx] >> (byteIdx * 8u)) & 0xFFu;
}

void main() {
    uint x = gl_GlobalInvocationID.x;
    uint y = gl_GlobalInvocationID.y;
    if (x >= params.width || y >= params.height) return;

    uint c = readPixel(x, y);
    uint l = (x > 0u) ? readPixel(x-1u, y) : c;
    uint r = (x < params.width-1u) ? readPixel(x+1u, y) : c;
    uint u = (y > 0u) ? readPixel(x, y-1u) : c;
    uint d = (y < params.height-1u) ? readPixel(x, y+1u) : c;

    uint rX = params.bayerPhase & 1u;
    uint rY = (params.bayerPhase >> 1) & 1u;
    uint px = x & 1u, py = y & 1u;

    int R, G, B;
    if (py == rY && px == rX) {
        R = int(c); G = int((l+r+u+d)/4u);
        uint ul = (x>0u && y>0u) ? readPixel(x-1u,y-1u) : u;
        uint ur = (x<params.width-1u && y>0u) ? readPixel(x+1u,y-1u) : u;
        uint dl = (x>0u && y<params.height-1u) ? readPixel(x-1u,y+1u) : d;
        uint dr = (x<params.width-1u && y<params.height-1u) ? readPixel(x+1u,y+1u) : d;
        B = int((ul+ur+dl+dr)/4u);
    } else if (py != rY && px != rX) {
        B = int(c); G = int((l+r+u+d)/4u);
        uint ul = (x>0u && y>0u) ? readPixel(x-1u,y-1u) : u;
        uint ur = (x<params.width-1u && y>0u) ? readPixel(x+1u,y-1u) : u;
        uint dl = (x>0u && y<params.height-1u) ? readPixel(x-1u,y+1u) : d;
        uint dr = (x<params.width-1u && y<params.height-1u) ? readPixel(x+1u,y+1u) : d;
        R = int((ul+ur+dl+dr)/4u);
    } else {
        G = int(c);
        if (py == rY) { R = int((l+r)/2u); B = int((u+d)/2u); }
        else           { B = int((l+r)/2u); R = int((u+d)/2u); }
    }

    if (params.doIsp != 0u) {
        R = clamp((R * int(params.wbR)) >> 8, 0, 255);
        G = clamp((G * int(params.wbG)) >> 8, 0, 255);
        B = clamp((B * int(params.wbB)) >> 8, 0, 255);
        int rr = clamp((params.ccm[0]*R + params.ccm[1]*G + params.ccm[2]*B) >> 10, 0, 255);
        int gg = clamp((params.ccm[3]*R + params.ccm[4]*G + params.ccm[5]*B) >> 10, 0, 255);
        int bb = clamp((params.ccm[6]*R + params.ccm[7]*G + params.ccm[8]*B) >> 10, 0, 255);
        R = int(gammaLookup(uint(rr)));
        G = int(gammaLookup(uint(gg)));
        B = int(gammaLookup(uint(bb)));
    }

    imageStore(outImg, ivec2(x, y), vec4(float(R)/255.0, float(G)/255.0, float(B)/255.0, 1.0));
}
)";

GlesIspPipeline::GlesIspPipeline()
    : mReady(false), mBufWidth(0), mBufHeight(0)
    , mDisplay(EGL_NO_DISPLAY), mContext(EGL_NO_CONTEXT), mSurface(EGL_NO_SURFACE)
    , mProgram(0), mInSSBO(0), mParamSSBO(0), mOutTex(0), mFbo(0)
    , mInSize(0) {}

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

bool GlesIspPipeline::process(const uint8_t *src, uint8_t *dst,
                               unsigned width, unsigned height,
                               uint32_t pixFmt) {
    if (!mReady) return false;

    /* Bind context once on capture thread, keep bound */
    if (eglGetCurrentContext() != mContext)
        eglMakeCurrent(mDisplay, mSurface, mSurface, mContext);

    bool is16 = (pixFmt == V4L2_PIX_FMT_SRGGB10 || pixFmt == V4L2_PIX_FMT_SGRBG10 ||
                 pixFmt == V4L2_PIX_FMT_SGBRG10 || pixFmt == V4L2_PIX_FMT_SBGGR10);

    size_t inSize = width * height * (is16 ? 2 : 1);
    size_t outSize = width * height * 4;

    /* Resize buffers if needed */
    if (mInSize < inSize || mBufWidth != width || mBufHeight != height) {
        if (mInSSBO) glDeleteBuffers(1, &mInSSBO);
        if (mOutTex) glDeleteTextures(1, &mOutTex);
        if (mFbo) glDeleteFramebuffers(1, &mFbo);

        glGenBuffers(1, &mInSSBO);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, mInSSBO);
        glBufferData(GL_SHADER_STORAGE_BUFFER, inSize, NULL, GL_STREAM_DRAW);

        /* Output texture + FBO for glReadPixels readback */
        glGenTextures(1, &mOutTex);
        glBindTexture(GL_TEXTURE_2D, mOutTex);
        glTexStorage2D(GL_TEXTURE_2D, 1, GL_RGBA8, width, height);

        glGenFramebuffers(1, &mFbo);
        glBindFramebuffer(GL_FRAMEBUFFER, mFbo);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, mOutTex, 0);

        GLenum fbStatus = glCheckFramebufferStatus(GL_FRAMEBUFFER);
        if (fbStatus != GL_FRAMEBUFFER_COMPLETE)
            ALOGE("FBO incomplete: 0x%x", fbStatus);

        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        mInSize = inSize;
        mBufWidth = width; mBufHeight = height;
    }

    int64_t t0 = nowMs();

    /* Upload input via mapped write (avoids driver-side copy in glBufferSubData) */
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, mInSSBO);
    void *inMap = glMapBufferRange(GL_SHADER_STORAGE_BUFFER, 0, inSize,
        GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
    if (inMap) {
        memcpy(inMap, src, inSize);
        glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
    } else {
        glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, inSize, src);
    }

    /* Fill params */
    IspParams params = {};
    params.width = width;
    params.height = height;
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

    /* Bind and dispatch */
    glUseProgram(mProgram);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, mInSSBO);
    glBindImageTexture(1, mOutTex, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA8);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, mParamSSBO);
    int64_t t1 = nowMs();

    glDispatchCompute((width + 15) / 16, (height + 15) / 16, 1);
    glMemoryBarrier(GL_FRAMEBUFFER_BARRIER_BIT | GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
    int64_t t2 = nowMs();

    /* Read back via FBO + glReadPixels (uses tiled memory + DMA path) */
    glBindFramebuffer(GL_READ_FRAMEBUFFER, mFbo);
    glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, dst);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
    int64_t t3 = nowMs();

    ALOGD("GLES: upload=%lld gpu=%lld readback=%lldms", t1 - t0, t2 - t1, t3 - t2);

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

void GlesIspPipeline::destroy() {
    if (!mReady && mDisplay == EGL_NO_DISPLAY)
        return;

    if (mDisplay != EGL_NO_DISPLAY) {
        eglMakeCurrent(mDisplay, EGL_NO_SURFACE, EGL_NO_SURFACE, mContext);
        if (mInSSBO) { glDeleteBuffers(1, &mInSSBO); mInSSBO = 0; }
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
