#ifndef VULKAN_INPUT_RING_H
#define VULKAN_INPUT_RING_H

#include <stddef.h>
#include <stdint.h>

#define VK_USE_PLATFORM_ANDROID_KHR
#include <vulkan/vulkan.h>

namespace android {

class VulkanDeviceState;

/* Ring of exportable VkBuffers that V4L2 writes Bayer frames into via
 * DMABUF. The fd for each slot comes from vkGetMemoryFdKHR (OPAQUE_FD)
 * and is handed to V4L2 through VIDIOC_QBUF(.m.fd=...). The ISP shader
 * then reads the slot directly through a descriptor binding — no CPU
 * memcpy on the hot path.
 *
 * Sized at configure time via ensureSize(); the slot count is fixed at
 * construction to match the V4L2 queue depth. */
class VulkanInputRing {
public:
    explicit VulkanInputRing(const VulkanDeviceState &dev);
    ~VulkanInputRing();

    void destroy();

    /* (Re)allocate every slot to hold `bytesPerSlot`. No-op when the
     * ring already matches; reallocates on size change (V4L2 resolution
     * change, pixel format change). Returns false on any Vulkan failure;
     * the ring is left empty. */
    bool ensureSize(size_t bytesPerSlot);

    int      slotCount() const { return kSlotCount; }
    size_t   slotSize()  const { return mSize; }

    /* Buffer handle for descriptor binding. VK_NULL_HANDLE before
     * ensureSize(). */
    VkBuffer buffer(int idx) const;

    /* OPAQUE_FD dma-buf fd for slot `idx`. Caller owns the fd and must
     * close it (or hand it to V4L2 which takes ownership). Returns -1
     * on failure. */
    int exportFd(int idx);

    /* Read-only CPU view of slot `idx` for frame analysis (gray-world
     * AWB sampling). Persistent mapping — not intended for upload, and
     * calling this from the frame-production path defeats the DMABUF
     * zero-copy contract. Returns NULL before ensureSize().
     *
     * Cache coherence: slots are allocated HOST_VISIBLE + HOST_CACHED
     * when available, so invalidateFromGpu() must be called before each
     * CPU read or the view will show stale cache lines. */
    void *mapped(int idx) const;
    void  invalidateFromGpu(int idx);

private:
    /* Must match V4L2DEVICE_BUF_COUNT — each slot is bound 1:1 to a V4L2
     * queue index. */
    static const int kSlotCount = V4L2DEVICE_BUF_COUNT;

    const VulkanDeviceState &mDev;
    VkBuffer       mBuf[kSlotCount];
    VkDeviceMemory mMem[kSlotCount];
    void          *mMap[kSlotCount];
    size_t         mSize;
};

}; /* namespace android */

#endif /* VULKAN_INPUT_RING_H */
