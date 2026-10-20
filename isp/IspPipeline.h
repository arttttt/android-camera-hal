#ifndef ISP_PIPELINE_H
#define ISP_PIPELINE_H

#include <stdint.h>
#include <stddef.h>

#include "CropRect.h"
#include "JpegSnapshot.h"
#include "mochacamera/Buffers.h"

namespace android {

/* The ISP backend, seen by each of its users only through the part
 * it uses. One backend implements them all (VulkanIspPipeline); the
 * interfaces are pure, so a missing piece is a build error rather than
 * a silent "return false".
 *
 * Threads: the frame recorder runs on the demosaic engine; the JPEG
 * ring on the finish and JPEG engines and wherever a frame is let go;
 * the YUV readback on the finish engine; the input ring on the capture
 * engine (and on the configuring thread for the buffer export); the
 * controls on the finish engine (the 3A tick) and at setup. */

/* Recording one frame: the demosaic, then one operation per output,
 * then a single submit.
 *
 * Demosaic happens once per frame; the resulting RGBA scratch is then
 * sampled / copied N times (one per output stream in the request).
 *
 *   beginFrame(...)    -> records demosaic into scratch
 *   blitToGralloc(...) -> records sampler-blit into an RGBA output
 *   blitToYuv(...)     -> records compute NV12 encode for a YUV output
 *   blitToJpegCpu(...) -> records a copy into a JPEG ring slot
 *   endFrame()         -> submits; release fences populated by blitTo*
 *                         are valid once it returns true.
 *
 * Single submit per frame: every operation shares the same VkSubmit, so
 * all outputs of one frame complete on the GPU at the same time.
 * Acquire fences are imported as binary VkSemaphores so the submit
 * waits on them GPU-side; the recording thread never blocks on
 * framework backpressure.
 *
 * On error inside any blitTo*, that output is not recorded; the frame
 * goes on. On error in beginFrame or endFrame, nothing is submitted. */
class IspFrameRecorder {
public:
    virtual ~IspFrameRecorder() = default;

    virtual bool beginFrame(unsigned srcW, unsigned srcH, uint32_t pixFmt,
                             int srcInputSlot) = 0;

    /* A sampler-blit of the scratch image into the output buffer (RGBA8
     * stream). Identity dst==src uses texelFetch in the shader;
     * otherwise hardware bilinear via sampler2D. acquireFence ownership
     * transfers to the implementation whatever the outcome
     * (closed/imported). The buffer is the frontend's NativeBuffer; the
     * backend knows what it is. */
    virtual bool blitToGralloc(mocha::camera::NativeBuffer *buffer,
                                unsigned dstW, unsigned dstH,
                                const CropRect &crop,
                                int acquireFence,
                                int *releaseFenceOut) = 0;

    /* A compute NV12 encode of the scratch image for the output buffer
     * (YCbCr_420_888 stream). Width must be a multiple of 4 and height a
     * multiple of 2. acquireFence semantics match blitToGralloc. */
    virtual bool blitToYuv(mocha::camera::NativeBuffer *buffer,
                            unsigned dstW, unsigned dstH,
                            const CropRect &crop,
                            int acquireFence,
                            int *releaseFenceOut) = 0;

    /* A vkCmdCopyImageToBuffer of the scratch image into a
     * backend-owned host-mapped RGBA8 buffer, picked from a small ring
     * so multiple BLOB requests can be in flight without overwriting
     * each other. On success populates *out with the buffer's CPU
     * pointer + dimensions + an opaque ringSlot handle, given back
     * through IspJpegRing. */
    virtual bool blitToJpegCpu(JpegSnapshot *out) = 0;

    /* Submit the recorded command buffer. Must be called once per
     * matching beginFrame, after all blitTo* calls.
     * submitFenceOut: [out, may be NULL] sync_fd that signals when the
     *                 submit completes; caller-owned. -1 if the backend
     *                 cannot export.
     * Returns false on submit failure; release fences from blitTo* are
     * invalid in that case. */
    virtual bool endFrame(int *submitFenceOut) = 0;

    /* The backend's device is gone (VK_ERROR_DEVICE_LOST): nothing it
     * records will run again, and the camera must report a device
     * error. Sticky until the backend is rebuilt. */
    virtual bool deviceLost() const = 0;
};

/* The ring of host-mapped RGBA copies blitToJpegCpu fills. */
class IspJpegRing {
public:
    virtual ~IspJpegRing() = default;

    /* Invalidate the CPU cache for the snapshot's buffer so a host read
     * after the GPU write sees coherent data. Called right before
     * libjpeg reads. */
    virtual void invalidateJpegSnapshot(const JpegSnapshot &snap) = 0;

    /* Mark the snapshot's ring slot free for reuse. Must be called
     * exactly once per successful blitToJpegCpu. Any thread. */
    virtual void releaseJpegSnapshot(const JpegSnapshot &snap) = 0;
};

/* The host-mapped NV12 buffer blitToYuv encodes into. */
class IspYuvReadback {
public:
    virtual ~IspYuvReadback() = default;

    /* Valid only after endFrame's fence has been reaped and
     * invalidateYuvForCpu() has been called. The buffer is shared across
     * the (at most) one in-flight frame. */
    virtual const uint8_t *yuvHostBuffer() const = 0;

    /* Invalidate the CPU cache for the YUV host buffer so a host read
     * after the GPU write sees coherent data. */
    virtual void invalidateYuvForCpu() = 0;
};

/* The backend's Bayer input ring, which the capture buffers are. */
class IspInputRing {
public:
    virtual ~IspInputRing() = default;

    /* Number of exportable Bayer input buffers the backend has
     * pre-allocated; 0 when the backend does not support DMABUF input. */
    virtual int inputBufferCount() const = 0;

    /* Export input buffer `idx` as a dma-buf fd, -1 on failure. Caller
     * owns the fd; the backend keeps its own reference. */
    virtual int exportInputBufferFd(int idx) = 0;

    /* Read-only host pointer to the raw Bayer content of input slot
     * `slot`. Valid for the lifetime of the backend; the content is
     * defined while a BayerLease on the slot is held and after
     * invalidateBayer(slot). NULL without a CPU-mapped input ring. */
    virtual const void *bayerHost(int slot) const = 0;

    /* Invalidate the CPU cache range backing bayerHost(slot) so a host
     * read after the V4L2 write sees coherent data. */
    virtual void invalidateBayer(int slot) = 0;
};

/* What 3A and the setup tell the demosaic shader. Taken by the next
 * frame recorded. */
class IspControls {
public:
    virtual ~IspControls() = default;

    /* AWB gains (Q8: 256 = 1.0x). */
    virtual void setWbGains(unsigned r, unsigned g, unsigned b) = 0;

    /* CCM matrix (Q10: 1024 = 1.0x), 3x3 row-major. The buffer is the
     * caller's and is read at every frame, so the caller rewrites it in
     * place to change the matrix. */
    virtual void setCcm(const int16_t *ccm) = 0;

    /* Optical-black bias, in the sensor's native range (10-bit or 8-bit
     * depending on pixel format). Applied per pixel before demosaic so
     * shadows hit true 0 and full 255 dynamic range is preserved. */
    virtual void setBlackLevel(uint32_t v) = 0;

    /* ISP processing on, or demosaic only. */
    virtual void setEnabled(bool en) = 0;
};

/* The whole backend, for its owner. */
class IspPipeline : public IspFrameRecorder,
                    public IspJpegRing,
                    public IspYuvReadback,
                    public IspInputRing,
                    public IspControls {
public:
    virtual bool init() = 0;
    virtual void destroy() = 0;

    /* Size the backend for the capture and warm its shaders before the
     * first real frame. */
    virtual void prewarm(unsigned width, unsigned height, uint32_t pixFmt) = 0;

    /* Called by closeDevice after the engines have stopped and the GPU
     * drained. The backend releases per-session references -- notably
     * cached bindings into framework-owned gralloc buffers, which become
     * invalid the moment the session ends. Core Vulkan resources
     * (device, scratch image, input ring, shaders) stay, since the
     * backend survives close -> reopen. */
    virtual void onSessionClose() = 0;
};

/* Construct the ISP backend. Caller owns the returned instance and must
 * call init() before use. */
IspPipeline *createIspPipeline();

}; /* namespace android */

#endif // ISP_PIPELINE_H
