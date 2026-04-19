#ifndef V4L2DEVICE_H
#define V4L2DEVICE_H

#include <linux/videodev2.h>
#include <sys/poll.h>
#include <stdint.h>
#include <utils/Vector.h>
#include <utils/Log.h>
#include <utils/Mutex.h>
#include <utils/Timers.h>

#ifndef V4L2DEVICE_BUF_COUNT
# error "V4L2DEVICE_BUF_COUNT must be defined via -DV4L2DEVICE_BUF_COUNT=<N> in Android.mk"
#endif

namespace android {

class V4l2Device
{
public:
    struct Resolution {
        unsigned width;
        unsigned height;
    };

    class VBuffer {
    public:
        uint8_t *buf;     /* CPU-visible pointer (MMAP mode); NULL in DMABUF mode */
        uint32_t len;
        uint32_t pixFmt;
        int      index;   /* V4L2 queue index, valid in both modes */

    private:
        VBuffer(): buf(NULL), len(0), pixFmt(0), index(-1) {}
        ~VBuffer();

        bool map(int fd, unsigned offset, unsigned len, uint32_t pixelFormat);
        void unmap();

        friend class V4l2Device;
    };

    V4l2Device(const char *devNode = "/dev/video0");
    ~V4l2Device();

    uint32_t pixelFormat() const { return mPixelFormat; }
    const Vector<V4l2Device::Resolution> & availableResolutions();
    V4l2Device::Resolution sensorResolution();

    /* Shortest frame period the driver reports for (width, height) at the
     * current pixel format, in nanoseconds. Returns 0 if the driver gives
     * no framerate info for that mode. */
    int64_t minFrameDurationNs(unsigned width, unsigned height);

    bool setResolution(unsigned width, unsigned height);
    V4l2Device::Resolution resolution();

    bool connect();
    bool disconnect();
    bool isConnected() const { return mFd >= 0; }

    bool setStreaming(bool enable);
    bool isStreaming() const { return mStreaming; }

    const VBuffer * readLock();
    bool unlock(const VBuffer *buf);

    /* Switch capture memory model from the default V4L2_MEMORY_MMAP to
     * V4L2_MEMORY_DMABUF, using the caller-owned dma-buf fds as capture
     * targets. Each fd i is bound to V4L2 queue index i. Must be called
     * BEFORE setResolution()/connect()'s buffer allocation; takes effect
     * the next time buffers are (re-)allocated. Pass count=0 to revert. */
    void setDmaBufFds(const int *fds, int count);

    bool setControl(uint32_t id, int32_t value);
    bool queryControl(uint32_t id, int32_t *min, int32_t *max, int32_t *def);

    /* Focuser subdev control */
    bool openFocuser(const char *subdevPath);
    bool setFocusPosition(int32_t position);
    void closeFocuser();

private:
    bool queueBuffer(unsigned id);
    int dequeueBuffer();

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
    uint32_t mPixelFormat;
    uint32_t mMemoryType;                   /* V4L2_MEMORY_MMAP or _DMABUF */
    int mDmaBufFds[V4L2DEVICE_BUF_COUNT];   /* for DMABUF mode; -1 = unused */

    /* Deferred-return queue for DMABUF mode: unlock() stashes the index here,
     * and readLock() flushes the stash back to the kernel via VIDIOC_QBUF at
     * the start of the next frame — by then the previous frame's GPU dispatch
     * has drained (guaranteed by the ISP's WaitForFences), so VI can safely
     * overwrite the buffer. mPendingQbufHead is -1 when the stash is empty. */
    int mPendingQbuf[V4L2DEVICE_BUF_COUNT];
    int mPendingQbufCount;
    Vector<V4l2Device::Resolution> mAvailableResolutions;
    struct v4l2_format mFormat;
    VBuffer mBuf[V4L2DEVICE_BUF_COUNT];
    struct pollfd mPFd;
    int mFocuserFd;

#if V4L2DEVICE_FPS_LIMIT > 0
    nsecs_t mLastTimestamp;
#endif
};

}; /* namespace android */

#endif // V4L2DEVICE_H
