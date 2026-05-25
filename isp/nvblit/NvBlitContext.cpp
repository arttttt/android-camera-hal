#define LOG_TAG "Cam-NvBlit"

#include "NvBlitContext.h"

#include <dlfcn.h>
#include <stddef.h>
#include <string.h>

#include <utils/Log.h>

namespace android {

namespace {

/* libnvblit ABI mirror — minimal subset needed for a src→dst blit
 * with optional rects. Layout follows the public JXD nvblit.h. The
 * vk_storage_probe and nvblit_probe binaries exercise this same
 * layout against the running blob and pass. */

struct NvBlitRectF {
    float left, top, right, bottom;
};

struct NvBlitState {
    uint32_t       ValidFields;
    uint32_t       Flags;
    const void    *SrcSurface;   /* buffer_handle_t */
    const void    *DstSurface;
    int            SrcSync;
    int            DstSync;
    NvBlitRectF    SrcRect;
    NvBlitRectF    DstRect;
    int32_t        Transform;
    int32_t        Filter;
    uint32_t       SrcColor;
    int32_t        SrcColorFormat;
    int32_t        SrcColorSpace;
    int32_t        DstColorSpace;
    int32_t        ColorProfileType;
};

struct NvBlitResult {
    int            ReleaseSync;
    struct { int x, y; } DstOffset;
    const char    *ErrorString;
};

enum {
    NvBlitState_SrcSurface = 1 << 0,
    NvBlitState_DstSurface = 1 << 1,
    NvBlitState_SrcRect    = 1 << 2,
    NvBlitState_DstRect    = 1 << 3,
};

/* Library search paths — same order as everywhere else in the project:
 * vendor first (the canonical Tegra location), then /system fallback,
 * then plain "libnvblit.so" if the linker can resolve. */
const char *kLibPaths[] = {
    "/system/vendor/lib/libnvblit.so",
    "/vendor/lib/libnvblit.so",
    "/system/lib/libnvblit.so",
    "libnvblit.so",
};

} /* namespace */

NvBlitContext::NvBlitContext()
    : mLib(nullptr)
    , mCtx(nullptr)
    , mOpen(nullptr)
    , mBlit(nullptr)
    , mClose(nullptr) {}

NvBlitContext::~NvBlitContext() {
    if (mCtx && mClose) mClose(mCtx);
    if (mLib)           dlclose(mLib);
    mCtx = nullptr;
    mLib = nullptr;
}

bool NvBlitContext::init() {
    if (mCtx) return true;   /* already initialised */

    for (const char *path : kLibPaths) {
        mLib = dlopen(path, RTLD_NOW | RTLD_LOCAL);
        if (mLib) {
            ALOGD("loaded %s", path);
            break;
        }
    }
    if (!mLib) {
        ALOGE("dlopen libnvblit.so failed: %s", dlerror());
        return false;
    }

    mOpen  = (pfn_NvBlitOpen) dlsym(mLib, "NvBlitOpen");
    mBlit  = (pfn_NvBlit)     dlsym(mLib, "NvBlit");
    mClose = (pfn_NvBlitClose)dlsym(mLib, "NvBlitClose");
    if (!mOpen || !mBlit || !mClose) {
        ALOGE("dlsym failed (open=%p blit=%p close=%p)", mOpen, mBlit, mClose);
        dlclose(mLib);
        mLib = nullptr;
        return false;
    }

    int32_t e = mOpen(&mCtx);
    if (e != 0 || !mCtx) {
        ALOGE("NvBlitOpen failed: %d", (int)e);
        dlclose(mLib);
        mLib = nullptr;
        mCtx = nullptr;
        return false;
    }

    ALOGD("ready ctx=%p", mCtx);
    return true;
}

bool NvBlitContext::blit(buffer_handle_t src, const Rect *srcRect, int srcFence,
                         buffer_handle_t dst, const Rect *dstRect, int dstFence,
                         int *releaseFenceOut) {
    if (releaseFenceOut) *releaseFenceOut = -1;
    if (!mCtx || !src || !dst) return false;

    NvBlitState state;
    memset(&state, 0, sizeof(state));
    state.ValidFields = NvBlitState_SrcSurface | NvBlitState_DstSurface;
    state.SrcSurface  = src;
    state.DstSurface  = dst;
    state.SrcSync     = srcFence;
    state.DstSync     = dstFence;

    if (srcRect) {
        state.ValidFields |= NvBlitState_SrcRect;
        state.SrcRect.left   = (float)srcRect->x;
        state.SrcRect.top    = (float)srcRect->y;
        state.SrcRect.right  = (float)(srcRect->x + srcRect->w);
        state.SrcRect.bottom = (float)(srcRect->y + srcRect->h);
    }
    if (dstRect) {
        state.ValidFields |= NvBlitState_DstRect;
        state.DstRect.left   = (float)dstRect->x;
        state.DstRect.top    = (float)dstRect->y;
        state.DstRect.right  = (float)(dstRect->x + dstRect->w);
        state.DstRect.bottom = (float)(dstRect->y + dstRect->h);
    }

    NvBlitResult result;
    memset(&result, 0, sizeof(result));
    int32_t e = mBlit(mCtx, &state, &result);
    if (e != 0) {
        ALOGE("NvBlit failed: %d errStr=%s", (int)e,
              result.ErrorString ? result.ErrorString : "(null)");
        return false;
    }

    if (releaseFenceOut) *releaseFenceOut = result.ReleaseSync;
    return true;
}

} /* namespace android */
