#ifndef VULKAN_GRALLOC_CACHE_H
#define VULKAN_GRALLOC_CACHE_H

#include <cutils/native_handle.h>
#include <unordered_map>

#define VK_USE_PLATFORM_ANDROID_KHR
#include <vulkan/vulkan.h>

struct ANativeWindowBuffer;

namespace android {

class VulkanDeviceState;

/* Per-gralloc-handle cache of Vulkan wrappers (VkImage + VkImageView +
 * VkFramebuffer) bound to the same backing memory gralloc already owns.
 *
 * Tegra's only blocklinear-aware write surface from Vulkan is the
 * fragment-shader ROP — which means zero-copy output needs an
 * ANB-bound VkImage the colour attachment can target. The framework
 * may hand the same buffer_handle_t to multiple frames, so wrappers
 * are cached by handle pointer. */
class VulkanGrallocCache {
public:
    struct Entry {
        VkImage       image;
        VkImageView   view;
        VkFramebuffer framebuffer;
        /* Stale-detection identity. Camera2's BufferQueue hands back the
         * same `native_handle_t *` pointer for buffers whose underlying
         * gralloc allocation has been replaced; the cached VkImage is
         * still bound to the old import and renders into freed memory.
         * `handleHash` covers all ints in the handle (offsets, strides,
         * format flags) and `dmabufInode` catches the case where the fd
         * value matches but the kernel reused it for a different
         * dma-buf. Mismatch on either invalidates the entry. */
        uint32_t      handleHash;
        uint64_t      dmabufInode;
        /* Frame (tick count) the entry was last handed out on. */
        uint64_t      lastUsed;
    };

    explicit VulkanGrallocCache(const VulkanDeviceState &dev);
    ~VulkanGrallocCache();

    /* Render pass used for framebuffer creation. Must be set before
     * getOrCreate() and must outlive every cached entry; caller keeps
     * ownership. */
    void setRenderPass(VkRenderPass rp);

    /* Look up or create the wrappers for this gralloc buffer. On cache
     * miss allocates image/view/framebuffer and stores them; `outEntry`
     * points into the cache. Returns false on any Vulkan failure. */
    bool getOrCreate(ANativeWindowBuffer *anwb,
                     unsigned width, unsigned height,
                     Entry **outEntry);

    /* Destroy every cached wrapper and clear the map. Safe to call
     * before the device teardown — it's a no-op if the device is
     * already gone. */
    void clear();

    /* Called once per frame, after the frame's slot fence has been
     * waited on. Destroys the wrappers of buffers not handed out for
     * kIdleFrames frames: the framework replaces a session's buffers
     * whenever the stream set changes, and the cache would otherwise
     * keep one VkImage per buffer ever seen until the session closes.
     * kIdleFrames must stay above the pipeline's slot count, so no
     * submit still in flight refers to an evicted entry. */
    static constexpr uint64_t kIdleFrames = 64;
    void tick();

private:
    const VulkanDeviceState                           &mDev;
    VkRenderPass                                       mRenderPass;
    std::unordered_map<const native_handle_t *, Entry> mEntries;
    uint64_t                                           mFrame;

    void destroyEntry(Entry &e);
};

}; /* namespace android */

#endif /* VULKAN_GRALLOC_CACHE_H */
