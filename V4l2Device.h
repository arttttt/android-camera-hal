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
# define V4L2DEVICE_BUF_COUNT 4
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
        uint8_t *buf;
        uint32_t len;
        uint32_t pixFmt;
        int dmabufFd;  /* EXPBUF fd for zero-copy GPU access */

    private:
        VBuffer(): buf(NULL), len(0), dmabufFd(-1) {}
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

    bool setResolution(unsigned width, unsigned height);
    V4l2Device::Resolution resolution();

    bool connect();
    bool disconnect();
    bool isConnected() const { return mFd >= 0; }

    bool setStreaming(bool enable);
    bool isStreaming() const { return mStreaming; }

    const VBuffer * readLock();
    bool unlock(const VBuffer *buf);

    bool setControl(uint32_t id, int32_t value);

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
