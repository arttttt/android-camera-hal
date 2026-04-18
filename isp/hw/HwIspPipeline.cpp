#define LOG_TAG "Cam-HwIsp"

#include <utils/Log.h>
#include <dlfcn.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/ioctl.h>

#include "HwIspPipeline.h"

namespace android {

/* ---- nvmap ioctls ---- */
#define NVMAP_IOC_MAGIC 'N'

struct nvmap_create_handle {
    union { uint32_t id; uint32_t size; int32_t fd; };
    uint32_t handle;
};
struct nvmap_alloc_handle {
    uint32_t handle; uint32_t heap_mask; uint32_t flags; uint32_t align;
};
struct nvmap_rw_handle {
    unsigned long addr; uint32_t handle; uint32_t offset;
    uint32_t elem_size; uint32_t hmem_stride; uint32_t user_stride; uint32_t count;
};
struct nvmap_pin_handle {
    uint32_t handles; unsigned long addr; uint32_t count;
};

#define NVMAP_IOC_CREATE   _IOWR(NVMAP_IOC_MAGIC, 0, struct nvmap_create_handle)
#define NVMAP_IOC_ALLOC    _IOW(NVMAP_IOC_MAGIC, 3, struct nvmap_alloc_handle)
#define NVMAP_IOC_FREE     _IO(NVMAP_IOC_MAGIC, 4)
#define NVMAP_IOC_WRITE    _IOW(NVMAP_IOC_MAGIC, 6, struct nvmap_rw_handle)
#define NVMAP_IOC_READ     _IOW(NVMAP_IOC_MAGIC, 7, struct nvmap_rw_handle)
#define NVMAP_IOC_PIN_MULT _IOWR(NVMAP_IOC_MAGIC, 10, struct nvmap_pin_handle)
#define NVMAP_HEAP_IOVMM   (1 << 30)
#define NVMAP_HANDLE_WRITE_COMBINE 2

/* ---- nvhost ioctls ---- */
#define NVHOST_IOCTL_MAGIC 'H'

struct nvhost_get_param_arg { uint32_t param; uint32_t value; };
struct nvhost_syncpt_incr { uint32_t syncpt_id; uint32_t syncpt_incrs; };
struct nvhost_cmdbuf { uint32_t mem; uint32_t offset; uint32_t words; };
struct nvhost_reloc { uint32_t cmdbuf_mem; uint32_t cmdbuf_offset; uint32_t target; uint32_t target_offset; };
struct nvhost_reloc_shift { uint32_t shift; };
struct nvhost_fence { uint32_t syncpt_id; uint32_t value; };
struct nvhost_ctrl_syncpt_waitex_args { uint32_t id; uint32_t thresh; int32_t timeout; uint32_t value; };

struct nvhost32_submit_args {
    uint32_t submit_version; uint32_t num_syncpt_incrs; uint32_t num_cmdbufs;
    uint32_t num_relocs; uint32_t num_waitchks; uint32_t timeout;
    uint32_t syncpt_incrs; uint32_t cmdbufs; uint32_t relocs;
    uint32_t reloc_shifts; uint32_t waitchks; uint32_t waitbases;
    uint32_t class_ids; uint32_t pad[2]; uint32_t fences; uint32_t fence;
} __attribute__((packed));

#define NVHOST_IOCTL_CHANNEL_GET_SYNCPOINT _IOWR(NVHOST_IOCTL_MAGIC, 16, struct nvhost_get_param_arg)
#define NVHOST32_IOCTL_CHANNEL_SUBMIT _IOWR(NVHOST_IOCTL_MAGIC, 15, struct nvhost32_submit_args)
#define NVHOST_IOCTL_CTRL_SYNCPT_WAITEX _IOWR(NVHOST_IOCTL_MAGIC, 6, struct nvhost_ctrl_syncpt_waitex_args)

/* Host1X opcodes */
#define OP_SETCLASS(c,o,m) ((0<<28)|((o)<<16)|((c)<<6)|(m))
#define OP_INCR(o,n)       ((1<<28)|((o)<<16)|(n))
#define OP_NONINCR(o,n)    ((2<<28)|((o)<<16)|(n))
#define ISP_CLASS 0x32

/* ISP output format */
#define ISP_FMT_R8G8B8A8 0x43

/* ---- nvmap helpers ---- */

uint32_t HwIspPipeline::nvmapCreate(uint32_t size) {
    struct nvmap_create_handle ch = {};
    ch.size = size;
    if (ioctl(mNvmapFd, NVMAP_IOC_CREATE, &ch) < 0) return 0;
    return ch.handle;
}

int HwIspPipeline::nvmapAlloc(uint32_t handle) {
    struct nvmap_alloc_handle ah = { handle, NVMAP_HEAP_IOVMM, NVMAP_HANDLE_WRITE_COMBINE, 4096 };
    return ioctl(mNvmapFd, NVMAP_IOC_ALLOC, &ah);
}

int HwIspPipeline::nvmapWrite(uint32_t handle, uint32_t offset, const void *data, uint32_t size) {
    struct nvmap_rw_handle rw = { (unsigned long)data, handle, offset, size, size, size, 1 };
    return ioctl(mNvmapFd, NVMAP_IOC_WRITE, &rw);
}

int HwIspPipeline::nvmapRead(uint32_t handle, uint32_t offset, void *data, uint32_t size) {
    struct nvmap_rw_handle rw = { (unsigned long)data, handle, offset, size, size, size, 1 };
    return ioctl(mNvmapFd, NVMAP_IOC_READ, &rw);
}

uint32_t HwIspPipeline::nvmapPin(uint32_t handle) {
    struct nvmap_pin_handle ph = { handle, 0, 1 };
    if (ioctl(mNvmapFd, NVMAP_IOC_PIN_MULT, &ph) < 0) return 0;
    return (uint32_t)ph.addr;
}

void HwIspPipeline::nvmapFree(uint32_t handle) {
    if (handle) ioctl(mNvmapFd, NVMAP_IOC_FREE, handle);
}

/* ---- Constructor / Destructor ---- */

HwIspPipeline::HwIspPipeline()
    : mReady(false)
    , mNvmapFd(-1), mIspFd(-1), mCtrlFd(-1)
    , mIspHandle(NULL), mHwSettings(NULL)
    , mLibIsp(NULL), mPIspClose(NULL), mPHwDestroy(NULL)
    , mSpMemory(0)
    , mInHandle(0), mOutHandle(0), mStatsHandle(0), mCmdHandle(0)
    , mInIova(0), mOutIova(0), mStatsIova(0)
    , mBufWidth(0), mBufHeight(0)
    , mInSize(0), mOutSize(0) {
}

HwIspPipeline::~HwIspPipeline() {
    destroy();
}

/* ---- init / destroy ---- */

bool HwIspPipeline::init() {
    typedef uint32_t (*NvRmOpenNew_t)(void **);
    typedef uint32_t (*NvIspOpen_t)(void *, uint32_t, void **);
    typedef uint32_t (*NvIspHwSettingsCreate_t)(void *, void **);
    typedef uint32_t (*NvIspHwSettingsApply_t)(void *);

    mNvmapFd = open("/dev/nvmap", O_RDWR | O_SYNC);
    if (mNvmapFd < 0) { ALOGE("HwIsp: open nvmap failed"); return false; }

    mCtrlFd = open("/dev/nvhost-ctrl", O_RDWR);
    if (mCtrlFd < 0) { ALOGE("HwIsp: open nvhost-ctrl failed"); close(mNvmapFd); return false; }

    /* dlopen ISP blobs */
    dlopen("libnvos.so", RTLD_NOW | RTLD_GLOBAL);
    void *libNvrm = dlopen("libnvrm.so", RTLD_NOW | RTLD_GLOBAL);
    dlopen("libnvrm_graphics.so", RTLD_NOW | RTLD_GLOBAL);
    mLibIsp = dlopen("libnvisp_v3.so", RTLD_NOW | RTLD_GLOBAL);
    if (!mLibIsp) { ALOGE("HwIsp: %s", dlerror()); return false; }

    NvRmOpenNew_t pRmOpen = (NvRmOpenNew_t)dlsym(libNvrm, "NvRmOpenNew");
    NvIspOpen_t pIspOpen = (NvIspOpen_t)dlsym(mLibIsp, "NvIspOpen");
    mPIspClose = (NvIspClose_t)dlsym(mLibIsp, "NvIspClose");
    NvIspHwSettingsCreate_t pHwCreate = (NvIspHwSettingsCreate_t)dlsym(mLibIsp, "NvIspHwSettingsCreate");
    NvIspHwSettingsApply_t pHwApply = (NvIspHwSettingsApply_t)dlsym(mLibIsp, "NvIspHwSettingsApply");
    mPHwDestroy = (NvIspHwSettingsDestroy_t)dlsym(mLibIsp, "NvIspHwSettingsDestroy");

    if (!pRmOpen || !pIspOpen || !pHwCreate || !pHwApply) {
        ALOGE("HwIsp: missing symbols"); return false;
    }

    void *hRm = NULL;
    pRmOpen(&hRm);

    uint32_t err = pIspOpen(hRm, 1, &mIspHandle);
    if (err || !mIspHandle) { ALOGE("HwIsp: NvIspOpen err=0x%x", err); return false; }

    pHwCreate(mIspHandle, &mHwSettings);

    /* Strip streaming trigger from calibration gather */
    setenv("NVRM_SHIM_STRIP", "1", 1);
    err = pHwApply(mHwSettings);
    unsetenv("NVRM_SHIM_STRIP");

    if (err) { ALOGE("HwIsp: HwSettingsApply err=0x%x", err); return false; }

    /* Extract ISP channel fd from blob context */
    uint32_t *ctx = (uint32_t *)mIspHandle;
    void *nrmChannel = (void *)(uintptr_t)ctx[3];
    mIspFd = *(int *)nrmChannel;

    /* Get syncpoint */
    struct nvhost_get_param_arg gsp = {};
    gsp.param = 0;
    ioctl(mIspFd, NVHOST_IOCTL_CHANNEL_GET_SYNCPOINT, &gsp);
    mSpMemory = gsp.value;

    /* Pre-allocate command and stats buffers */
    mStatsHandle = nvmapCreate(262144);
    mCmdHandle = nvmapCreate(16384);
    if (!mStatsHandle || !mCmdHandle) { ALOGE("HwIsp: alloc failed"); return false; }
    nvmapAlloc(mStatsHandle);
    nvmapAlloc(mCmdHandle);
    mStatsIova = nvmapPin(mStatsHandle);

    ALOGI("HwIsp: initialized, isp_fd=%d syncpt=%u", mIspFd, mSpMemory);
    mReady = true;
    return true;
}

void HwIspPipeline::destroy() {
    if (!mReady) return;

    if (mHwSettings && mPHwDestroy) mPHwDestroy(mHwSettings);
    if (mIspHandle && mPIspClose) mPIspClose(mIspHandle);
    mIspHandle = NULL;
    mHwSettings = NULL;

    nvmapFree(mInHandle);
    nvmapFree(mOutHandle);
    nvmapFree(mStatsHandle);
    nvmapFree(mCmdHandle);
    mInHandle = mOutHandle = mStatsHandle = mCmdHandle = 0;

    if (mCtrlFd >= 0) close(mCtrlFd);
    if (mNvmapFd >= 0) close(mNvmapFd);
    mCtrlFd = mNvmapFd = -1;
    mReady = false;
}

/* ---- submit one strip ---- */

bool HwIspPipeline::submitStrip(int strip, int numStrips,
                                 uint32_t inH, uint32_t outH,
                                 uint32_t cmdH, uint32_t statsH) {
    unsigned W = mBufWidth;
    unsigned H = mBufHeight;
    unsigned stripW = W / numStrips;
    int stripX = strip * stripW;
    int inOff = stripX * 2;         /* input: 16bpp (BG10) */
    int outOff = stripX * 4;        /* output: 32bpp (RGBA) */
    uint32_t outStride = W * 4;
    uint32_t inStride = W * 2;

    uint32_t cmd[128];
    int n = 0;
    int yReloc = -1, uReloc = -1, vReloc = -1, inReloc = -1, statsReloc = -1;

    /* Output config */
    cmd[n++] = OP_INCR(0xE00, 1);
    cmd[n++] = ((W - 1) & 0x3FFF) << 16;
    cmd[n++] = OP_INCR(0xE01, 1);
    cmd[n++] = ((H - 1) & 0x3FFF) << 16;
    cmd[n++] = OP_INCR(0xE02, 1);
    cmd[n++] = ISP_FMT_R8G8B8A8;
    cmd[n++] = OP_INCR(0xE03, 1);
    cmd[n++] = 0x00000000;

    /* Y surface */
    cmd[n++] = OP_INCR(0xE04, 3);
    yReloc = n;
    cmd[n++] = 0;
    cmd[n++] = 0x00000000;
    cmd[n++] = outStride;
    /* U surface — same buffer to avoid MC errors */
    cmd[n++] = OP_INCR(0xE07, 3);
    uReloc = n;
    cmd[n++] = 0;
    cmd[n++] = 0x00000000;
    cmd[n++] = outStride;
    /* V surface */
    cmd[n++] = OP_INCR(0xE0A, 3);
    vReloc = n;
    cmd[n++] = 0;
    cmd[n++] = 0x00000000;
    cmd[n++] = outStride;

    /* Processing block */
    cmd[n++] = OP_INCR(0x500, 6);
    cmd[n++] = 0; cmd[n++] = 0; cmd[n++] = 0;
    cmd[n++] = 0; cmd[n++] = 0;
    cmd[n++] = (H << 16) | W;

    /* Stats buffer */
    cmd[n++] = OP_SETCLASS(ISP_CLASS, 0, 0);
    cmd[n++] = OP_INCR(0x100, 4);
    statsReloc = n;
    cmd[n++] = 0;
    cmd[n++] = 0; cmd[n++] = 0; cmd[n++] = 0;

    /* Input */
    cmd[n++] = OP_INCR(0xE31, 1);
    cmd[n++] = (H << 16) | W;
    cmd[n++] = OP_INCR(0xE33, 1);
    cmd[n++] = 0x10200024;            /* 10-bit Bayer BGGR */
    cmd[n++] = OP_INCR(0xE34, 3);
    inReloc = n;
    cmd[n++] = 0;
    cmd[n++] = 0x00000000;
    cmd[n++] = inStride;
    cmd[n++] = OP_INCR(0xE32, 1);
    cmd[n++] = (W & 0x3FFF);
    cmd[n++] = OP_INCR(0xE30, 1);
    cmd[n++] = 1;

    /* ISP_ENABLE */
    cmd[n++] = OP_INCR(0x015, 1);
    cmd[n++] = 7;

    /* Syncpt incrs */
    uint32_t spStats = mSpMemory + 1;
    uint32_t spLoadv = mSpMemory + 2;
    cmd[n++] = OP_SETCLASS(ISP_CLASS, 0, 0);
    cmd[n++] = OP_NONINCR(0x000, 1);
    cmd[n++] = (4 << 8) | mSpMemory;
    cmd[n++] = OP_NONINCR(0x000, 1);
    cmd[n++] = (5 << 8) | spStats;
    cmd[n++] = OP_NONINCR(0x000, 1);
    cmd[n++] = (6 << 8) | spLoadv;

    /* Trigger reprocess */
    cmd[n++] = OP_SETCLASS(ISP_CLASS, 0, 0);
    cmd[n++] = OP_NONINCR(0x00C, 1);
    cmd[n++] = 0x0B;

    nvmapWrite(cmdH, 0, cmd, n * 4);

    /* Relocs */
    struct nvhost_reloc relocs[5];
    struct nvhost_reloc_shift shifts[5];
    int nr = 0;

    relocs[nr] = { cmdH, (uint32_t)(yReloc*4), outH, (uint32_t)outOff };
    shifts[nr++].shift = 0;
    relocs[nr] = { cmdH, (uint32_t)(uReloc*4), outH, 0 };
    shifts[nr++].shift = 0;
    relocs[nr] = { cmdH, (uint32_t)(vReloc*4), outH, 0 };
    shifts[nr++].shift = 0;
    relocs[nr] = { cmdH, (uint32_t)(inReloc*4), inH, (uint32_t)inOff };
    shifts[nr++].shift = 0;
    relocs[nr] = { cmdH, (uint32_t)(statsReloc*4), statsH, 0 };
    shifts[nr++].shift = 0;

    /* Submit */
    struct nvhost_cmdbuf cb = { cmdH, 0, (uint32_t)n };
    struct nvhost_syncpt_incr si = { mSpMemory, 1 };
    uint32_t classId = ISP_CLASS;
    struct nvhost_fence fence = { 0, 0 };

    struct nvhost32_submit_args sa;
    memset(&sa, 0, sizeof(sa));
    sa.submit_version = 0;
    sa.num_syncpt_incrs = 1;
    sa.num_cmdbufs = 1;
    sa.num_relocs = nr;
    sa.timeout = 5000;
    sa.syncpt_incrs = (uint32_t)(uintptr_t)&si;
    sa.cmdbufs = (uint32_t)(uintptr_t)&cb;
    sa.relocs = (uint32_t)(uintptr_t)relocs;
    sa.reloc_shifts = (uint32_t)(uintptr_t)shifts;
    sa.class_ids = (uint32_t)(uintptr_t)&classId;
    sa.fences = (uint32_t)(uintptr_t)&fence;

    if (ioctl(mIspFd, NVHOST32_IOCTL_CHANNEL_SUBMIT, &sa) < 0) {
        ALOGE("HwIsp: submit strip %d failed", strip);
        return false;
    }

    /* Wait */
    uint32_t thresh = sa.fence;
    struct nvhost_ctrl_syncpt_waitex_args wa = { mSpMemory, thresh, 5000, 0 };
    if (ioctl(mCtrlFd, NVHOST_IOCTL_CTRL_SYNCPT_WAITEX, &wa) < 0) {
        ALOGE("HwIsp: timeout strip %d (syncpt %u thresh %u)", strip, mSpMemory, thresh);
        return false;
    }

    return true;
}

/* ---- process frame ---- */

bool HwIspPipeline::process(const uint8_t *src, uint8_t *dst,
                             unsigned width, unsigned height,
                             uint32_t pixFmt) {
    if (!mReady) return false;

    /* (Re)allocate I/O buffers if dimensions changed */
    size_t inSize = width * height * 2;   /* BG10: 16bpp */
    size_t outSize = width * height * 4;  /* RGBA: 32bpp */

    if (width != mBufWidth || height != mBufHeight) {
        nvmapFree(mInHandle);
        nvmapFree(mOutHandle);
        mInHandle = nvmapCreate(inSize);
        mOutHandle = nvmapCreate(outSize);
        if (!mInHandle || !mOutHandle) { ALOGE("HwIsp: buf alloc failed"); return false; }
        nvmapAlloc(mInHandle);
        nvmapAlloc(mOutHandle);
        mInIova = nvmapPin(mInHandle);
        mOutIova = nvmapPin(mOutHandle);
        mBufWidth = width;
        mBufHeight = height;
        mInSize = inSize;
        mOutSize = outSize;
    }

    /* Upload raw input */
    const uint32_t chunk = 65536;
    for (size_t off = 0; off < inSize; off += chunk) {
        uint32_t sz = (inSize - off < chunk) ? inSize - off : chunk;
        nvmapWrite(mInHandle, off, src + off, sz);
    }

    /* Zero output */
    uint8_t zeros[65536] = {};
    for (size_t off = 0; off < outSize; off += chunk) {
        uint32_t sz = (outSize - off < chunk) ? outSize - off : chunk;
        nvmapWrite(mOutHandle, off, zeros, sz);
    }

    /* Submit 2 strips */
    int numStrips = 2;
    for (int s = 0; s < numStrips; s++) {
        if (!submitStrip(s, numStrips, mInHandle, mOutHandle, mCmdHandle, mStatsHandle)) {
            return false;
        }
    }

    /* Read output — ISP produces bytes as [A=0xFF, R, G, B] per pixel (format 0x43).
     * Convert to RGBA8888 that Android expects. */
    for (size_t off = 0; off < outSize; off += chunk) {
        uint32_t sz = (outSize - off < chunk) ? outSize - off : chunk;
        nvmapRead(mOutHandle, off, dst + off, sz);
    }

    return true;
}

}; /* namespace android */
