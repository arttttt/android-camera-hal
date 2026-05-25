/* Standalone smoke test for libnvblit.so on Tegra K1 Mocha.
 *
 * Purpose: verify that NvBlit can take two gralloc-allocated buffers
 * (RGBA src + NV12-like dst) and produce a HW VIC color-converting
 * blit. If this passes, we can hook the same call into the camera HAL
 * to replace the libyuv-based YUV encoder path with HW conversion.
 *
 * If this fails, the failure mode tells us:
 *   - dlopen/dlsym: blob path or symbol naming changed.
 *   - NvBlitOpen != 0: context creation failed (driver/permissions/state).
 *   - NvBlit != 0: state struct ABI doesn't match our header guess, or
 *     gralloc handle isn't usable by the VIC HW.
 *   - dst Y plane all-zero: blit recorded but didn't execute (sync wait
 *     bug, dst layout mismatch, source not yet committed).
 *
 * Run: adb push /data/local/tmp/nvblit_probe; adb shell /data/local/tmp/nvblit_probe
 */

#define LOG_TAG "nvblit_probe"

#include <dlfcn.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <hardware/gralloc.h>
#include <ui/GraphicBuffer.h>
#include <utils/Log.h>

using namespace android;

/* NvBlit ABI — extracted from JXD vendor headers
 * (tegra/graphics/2d/include/nvblit.h). Blob symbols on the device
 * (NvBlitOpen / NvBlit / NvBlitClose) match those names. The struct
 * layouts are our best guess; if NvBlit returns BadParameter or
 * doesn't write anything, the layout may have drifted in newer blobs. */
typedef int32_t  NvError;
typedef int      NvBlitSync;
typedef buffer_handle_t NvBlitSurface;

struct NvBlitContextRec;
typedef struct NvBlitContextRec NvBlitContext;

struct NvBlitRect {
    float left, top, right, bottom;
};

struct NvBlitState {
    uint32_t      ValidFields;
    uint32_t      Flags;
    NvBlitSurface SrcSurface;
    NvBlitSurface DstSurface;
    NvBlitSync    SrcSync;
    NvBlitSync    DstSync;
    NvBlitRect    SrcRect;
    NvBlitRect    DstRect;
    int32_t       Transform;
    int32_t       Filter;
    uint32_t      SrcColor;
    int32_t       SrcColorFormat;
    int32_t       SrcColorSpace;
    int32_t       DstColorSpace;
    int32_t       ColorProfileType;
};

struct NvBlitResult {
    NvBlitSync  ReleaseSync;
    struct { int x, y; } DstOffset;
    const char *ErrorString;
};

enum {
    NvBlitState_SrcSurface = 1 << 0,
    NvBlitState_DstSurface = 1 << 1,
    NvBlitState_SrcRect    = 1 << 2,
    NvBlitState_DstRect    = 1 << 3,
};

typedef NvError (*pfn_NvBlitOpen)(NvBlitContext **ctx);
typedef void    (*pfn_NvBlitClose)(NvBlitContext *ctx);
typedef NvError (*pfn_NvBlit)(NvBlitContext *ctx, const NvBlitState *state, NvBlitResult *result);

#define P(...)  do { fprintf(stderr, __VA_ARGS__); ALOGI(__VA_ARGS__); } while (0)
#define FAIL(...) do { fprintf(stderr, "FAIL: " __VA_ARGS__); fprintf(stderr, "\n"); ALOGE("FAIL: " __VA_ARGS__); return 1; } while (0)

int main() {
    void *h = dlopen("/vendor/lib/libnvblit.so", RTLD_NOW);
    if (!h) h = dlopen("/system/lib/libnvblit.so", RTLD_NOW);
    if (!h) h = dlopen("libnvblit.so", RTLD_NOW);
    if (!h) FAIL("dlopen libnvblit.so: %s", dlerror());
    P("dlopen libnvblit.so ok\n");

    pfn_NvBlitOpen  NvBlitOpen  = (pfn_NvBlitOpen) dlsym(h, "NvBlitOpen");
    pfn_NvBlit      NvBlit      = (pfn_NvBlit)     dlsym(h, "NvBlit");
    pfn_NvBlitClose NvBlitClose = (pfn_NvBlitClose)dlsym(h, "NvBlitClose");
    if (!NvBlitOpen || !NvBlit || !NvBlitClose)
        FAIL("dlsym: NvBlitOpen=%p NvBlit=%p NvBlitClose=%p", NvBlitOpen, NvBlit, NvBlitClose);
    P("dlsym ok\n");

    NvBlitContext *ctx = NULL;
    NvError e = NvBlitOpen(&ctx);
    if (e != 0) FAIL("NvBlitOpen returned %d", (int)e);
    P("NvBlitOpen ok ctx=%p\n", ctx);

    const uint32_t W = 64, H = 64;

    /* src: RGBA, CPU-writable so we can fill a pattern, also 2D-readable
     * so the HW VIC path is enabled. */
    sp<GraphicBuffer> src(new GraphicBuffer(W, H,
        HAL_PIXEL_FORMAT_RGBA_8888,
        GRALLOC_USAGE_HW_2D | GRALLOC_USAGE_SW_WRITE_OFTEN
            | GRALLOC_USAGE_SW_READ_OFTEN));
    if (src->initCheck() != NO_ERROR) FAIL("src GraphicBuffer alloc");
    P("src alloc ok handle=%p\n", src->getNativeBuffer()->handle);

    /* dst: NV12-like semi-planar YCrCb 4:2:0 — what the encoder consumes.
     * HW_VIDEO_ENCODER + SW_READ_OFTEN so we can inspect the bytes after
     * the blit. */
    sp<GraphicBuffer> dst(new GraphicBuffer(W, H,
        HAL_PIXEL_FORMAT_YCrCb_420_SP,
        GRALLOC_USAGE_HW_VIDEO_ENCODER | GRALLOC_USAGE_SW_READ_OFTEN));
    if (dst->initCheck() != NO_ERROR) FAIL("dst GraphicBuffer alloc");
    P("dst alloc ok handle=%p\n", dst->getNativeBuffer()->handle);

    /* Fill src with solid red so BT.601-limited Y ≈ 82, U ≈ 90, V ≈ 240. */
    {
        void *vp = NULL;
        if (src->lock(GRALLOC_USAGE_SW_WRITE_OFTEN, &vp) != NO_ERROR)
            FAIL("src lock for write");
        uint8_t *p = (uint8_t *)vp;
        for (uint32_t i = 0; i < W * H; i++) {
            p[i * 4 + 0] = 0xFF; /* R */
            p[i * 4 + 1] = 0x00; /* G */
            p[i * 4 + 2] = 0x00; /* B */
            p[i * 4 + 3] = 0xFF; /* A */
        }
        src->unlock();
    }

    /* Run the blit. Just set src + dst — rectangles default to full,
     * transform default = none. */
    NvBlitState state;
    memset(&state, 0, sizeof(state));
    state.ValidFields = NvBlitState_SrcSurface | NvBlitState_DstSurface;
    state.SrcSurface  = src->getNativeBuffer()->handle;
    state.DstSurface  = dst->getNativeBuffer()->handle;
    state.SrcSync     = -1;
    state.DstSync     = -1;

    NvBlitResult result;
    memset(&result, 0, sizeof(result));
    e = NvBlit(ctx, &state, &result);
    if (e != 0)
        FAIL("NvBlit returned %d errStr=%s", (int)e,
             result.ErrorString ? result.ErrorString : "(null)");
    P("NvBlit ok releaseSync=%d\n", result.ReleaseSync);

    /* CPU-wait the release fence so the dst is committed before we read it.
     * Simplest sync: small sleep — true fence wait isn't strictly needed for
     * a probe but is good practice. */
    if (result.ReleaseSync >= 0) {
        usleep(50000);   /* 50 ms is plenty for a 64x64 blit */
        close(result.ReleaseSync);
    }

    /* Inspect dst. We don't know the precise stride (gralloc may pad), so
     * just sample the first W*H bytes (Y plane min size) and check for
     * non-zero content. If blit really ran, average Y should be ~82. */
    {
        void *vp = NULL;
        if (dst->lock(GRALLOC_USAGE_SW_READ_OFTEN, &vp) != NO_ERROR)
            FAIL("dst lock for read");
        const uint8_t *p = (const uint8_t *)vp;
        long sum = 0;
        int  nz  = 0;
        int  yMin = 255, yMax = 0;
        for (uint32_t i = 0; i < W * H; i++) {
            if (p[i] != 0) nz++;
            sum += p[i];
            if (p[i] < yMin) yMin = p[i];
            if (p[i] > yMax) yMax = p[i];
        }
        P("dst Y plane: nonzero=%d/%d sum=%ld avg=%.1f min=%d max=%d\n",
          nz, W * H, sum, sum / (float)(W * H), yMin, yMax);
        P("Expected for solid red, BT.601-limited: avg Y ≈ 82\n");

        if (nz == 0) {
            dst->unlock();
            FAIL("dst Y plane is entirely zero — blit didn't write");
        }
        dst->unlock();
    }

    NvBlitClose(ctx);
    dlclose(h);
    P("PASS\n");
    return 0;
}
