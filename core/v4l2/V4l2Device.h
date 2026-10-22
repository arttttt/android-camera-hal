#ifndef V4L2DEVICE_H
#define V4L2DEVICE_H

#include <linux/videodev2.h>
#include <sys/poll.h>
#include <stdint.h>
#include <string>
#include <utils/Vector.h>
#include <utils/Log.h>
#include <utils/Mutex.h>
#include <utils/Timers.h>

#include "Resolution.h"

#ifndef V4L2DEVICE_BUF_COUNT
# error "V4L2DEVICE_BUF_COUNT must be defined via -DV4L2DEVICE_BUF_COUNT=<N> in Android.mk"
#endif

namespace android {

class V4l2Device
{
public:
    class VBuffer {
    public:
        uint8_t *buf;     /* CPU-visible pointer (MMAP mode); NULL in DMABUF mode */
        uint32_t len;
        uint32_t pixFmt;
        int      index;   /* V4L2 queue index, valid in both modes */

        /* When the capture thread dequeued the frame now in this slot
         * (systemTime, ~ the frame's end), and the interval to the
         * previous good frame (~ its length, more if frames were
         * dropped). Its exposure began about exposureStartNs(); 0 when
         * there is no previous frame to tell. Set by the capture thread
         * before the slot is handed out. */
        int64_t  dequeuedNs;
        int64_t  intervalNs;
        int64_t  exposureStartNs() const {
            return intervalNs > 0 ? dequeuedNs - intervalNs : 0;
        }

        /* What the driver said of the frame now in this slot; see
         * Dequeued. Set by the capture thread with dequeuedNs. */
        uint32_t sequence;
        int64_t  timestampNs;

    private:
        VBuffer(): buf(NULL), len(0), pixFmt(0), index(-1),
                   dequeuedNs(0), intervalNs(0),
                   sequence(0), timestampNs(0) {}
        ~VBuffer();

        bool map(int fd, unsigned offset, unsigned len, uint32_t pixelFormat);
        void unmap();

        friend class V4l2Device;
    };

    V4l2Device(const char *devNode = "/dev/video0");
    ~V4l2Device();

    uint32_t pixelFormat() const { return mPixelFormat; }
    const Vector<Resolution> & availableResolutions();
    Resolution sensorResolution();

    /* Shortest frame period the driver reports for (width, height) at the
     * current pixel format, in nanoseconds. Returns 0 if the driver gives
     * no framerate info for that mode. */
    int64_t minFrameDurationNs(unsigned width, unsigned height);

    /* Sorted, deduplicated set of FPS values the driver advertises across
     * every supported resolution at the current pixel format, gathered via
     * VIDIOC_ENUM_FRAMEINTERVALS. Empty when the driver reports no
     * intervals. Used by static metadata to populate
     * AE_AVAILABLE_TARGET_FPS_RANGES instead of a hardcoded list. */
    const Vector<int32_t> & availableFps();

    bool setResolution(unsigned width, unsigned height);
    Resolution resolution();

    bool connect();
    bool disconnect();
    bool isConnected() const { return mFd >= 0; }

    bool setStreaming(bool enable);
    bool isStreaming() const { return mStreaming; }

    /* Capture primitives for V4l2Source, which the capture engine's
     * thread drives: it polls fd() and dequeues without blocking. Only
     * while isStreaming(). */
    int  fd() const { return mFd; }
    /* What VIDIOC_DQBUF said of a frame besides its slot. */
    struct Dequeued {
        /* V4L2_BUF_FLAG_ERROR: the driver filled the slot but the
         * frame in it is damaged. */
        bool     corrupt = false;
        /* The driver's count of the buffers it has completed, damaged
         * ones included. A sensor frame the VI had no buffer armed for
         * is not counted. */
        uint32_t sequence = 0;
        /* When the frame's last line reached memory, CLOCK_MONOTONIC
         * (the queue's V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC), ns. */
        int64_t  timestampNs = 0;
    };

    /* EAGAIN -> -1; otherwise slot, and what the driver said of the
     * frame in `info` (optional). */
    int  dequeueBufferNonBlocking(Dequeued *info = nullptr);
    bool queueBuffer(unsigned id);           /* V4L2 QBUF; returns false on error */
    const VBuffer* slotAt(int id) const;     /* accessor into mBuf[id]; null on range error */

    /* Switch capture memory model from the default V4L2_MEMORY_MMAP to
     * V4L2_MEMORY_DMABUF, using the caller-owned dma-buf fds as capture
     * targets. Each fd i is bound to V4L2 queue index i. Must be called
     * BEFORE setResolution()/connect()'s buffer allocation; takes effect
     * the next time buffers are (re-)allocated. Pass count=0 to revert. */
    void setDmaBufFds(const int *fds, int count);

    bool setControl(uint32_t id, int32_t value);

    /* Batched write via VIDIOC_S_EXT_CTRLS. Preferred over looping
     * setControl() when multiple controls land together (exposure +
     * gain), because it is one ioctl and — on drivers that honour it —
     * atomic. No-op (returns true) when controls.count == 0. */
    bool setControls(const struct V4l2Controls &controls);

    bool queryControl(uint32_t id, int32_t *min, int32_t *max, int32_t *def);

    /* VIDIOC_G_CTRL: the current value -- for read-only, volatile
     * controls (the line time) whose QUERYCTRL range says nothing. */
    bool getControl(uint32_t id, int32_t *value);

    /* The capture node's path, for the media graph walk. */
    const char *devNode() const { return mDevNode; }

    /* The /dev/v4l-subdev* of the lens in front of this node's sensor,
     * from the media graph; empty for a fixed-focus module. Walked once,
     * on first ask; the node need not be connected. */
    const std::string &lensNode();
    bool hasLens() { return !lensNode().empty(); }

private:
    bool iocStreamOff();
    bool iocStreamOn();
    bool iocSFmt(unsigned width, unsigned height);
    bool iocReqBufs(unsigned *count);
    bool iocReleaseBufs();
    bool iocQueryBuf(unsigned id, unsigned *offset, unsigned *len);

    bool setResolutionAndAllocateBuffers(unsigned width, unsigned height);
    void cleanup();

    bool negotiatePixelFormat(int fd);

    int mFd;
    bool mConnected;
    bool mStreaming;
    const char *mDevNode;
    std::string mLensNode;
    bool mLensWalked = false;
    uint32_t mPixelFormat;
    uint32_t mMemoryType;                   /* V4L2_MEMORY_MMAP or _DMABUF */
    int mDmaBufFds[V4L2DEVICE_BUF_COUNT];   /* for DMABUF mode; -1 = unused */

    Vector<Resolution> mAvailableResolutions;
    Vector<int32_t>    mAvailableFps;
    struct v4l2_format mFormat;
    VBuffer mBuf[V4L2DEVICE_BUF_COUNT];
};

}; /* namespace android */

#endif // V4L2DEVICE_H
