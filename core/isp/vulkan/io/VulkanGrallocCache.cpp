#define LOG_TAG "Cam-VulkanGrallocCache"
#include <utils/Log.h>
#include <system/window.h>
#include <sys/stat.h>
#include <unistd.h>

#include "VulkanGrallocCache.h"
#include "runtime/VulkanDeviceState.h"
#include "runtime/loader/VulkanPfn.h"

namespace android {

namespace {

/* FNV-1a over every int the gralloc handle carries (fds + auxiliary
 * ints — offset, stride, format flags, etc). Any mutation in any of
 * those means the underlying allocation is different even when the
 * native_handle_t pointer is the same. */
uint32_t hashHandle(const native_handle_t *h) {
    uint32_t hash = 2166136261u;
    const int total = h->numFds + h->numInts;
    for (int i = 0; i < total; ++i) {
        const uint32_t v = (uint32_t)h->data[i];
        hash ^= v;
        hash *= 16777619u;
    }
    return hash;
}

/* Inode for the dma-buf the handle's first fd refers to. The kernel
 * recycles fd numbers across close()/open(), so the fd value alone
 * isn't unique — st_ino is. Returns 0 when the handle has no fd or
 * fstat fails (treated as "different from anything cached"). */
uint64_t dmabufInodeFor(const native_handle_t *h) {
    if (h->numFds <= 0) return 0;
    struct stat st;
    if (fstat(h->data[0], &st) != 0) return 0;
    return (uint64_t)st.st_ino;
}

} /* namespace */

VulkanGrallocCache::VulkanGrallocCache(const VulkanDeviceState &dev)
    : mDev(dev)
    , mRenderPass(VK_NULL_HANDLE)
    , mFrame(0) {}

VulkanGrallocCache::~VulkanGrallocCache() {
    clear();
}

void VulkanGrallocCache::setRenderPass(VkRenderPass rp) {
    mRenderPass = rp;
}

void VulkanGrallocCache::destroyEntry(Entry &e) {
    if (e.framebuffer)
        mDev.pfn()->DestroyFramebuffer(mDev.device(), e.framebuffer, NULL);
    if (e.view)
        mDev.pfn()->DestroyImageView(mDev.device(), e.view, NULL);
    if (e.image)
        mDev.pfn()->DestroyImage(mDev.device(), e.image, NULL);
}

void VulkanGrallocCache::clear() {
    if (!mDev.isReady() || mDev.device() == VK_NULL_HANDLE) {
        mEntries.clear();
        return;
    }
    for (auto &kv : mEntries)
        destroyEntry(kv.second);
    mEntries.clear();
}

void VulkanGrallocCache::tick() {
    ++mFrame;
    if (!mDev.isReady() || mDev.device() == VK_NULL_HANDLE) return;
    for (auto it = mEntries.begin(); it != mEntries.end(); ) {
        if (it->second.lastUsed + kIdleFrames < mFrame) {
            destroyEntry(it->second);
            it = mEntries.erase(it);
        } else {
            ++it;
        }
    }
}

bool VulkanGrallocCache::getOrCreate(ANativeWindowBuffer *anwb,
                                      unsigned width, unsigned height,
                                      Entry **outEntry) {
    const uint32_t curHash  = hashHandle(anwb->handle);
    const uint64_t curInode = dmabufInodeFor(anwb->handle);

    auto it = mEntries.find(anwb->handle);
    if (it != mEntries.end()) {
        Entry &e = it->second;
        if (e.handleHash == curHash && e.dmabufInode == curInode) {
            e.lastUsed = mFrame;
            *outEntry = &e;
            return true;
        }
        /* Same pointer, different underlying allocation — destroy the
         * stale entry and fall through to recreate. The caller is
         * responsible for ensuring no in-flight queue submit is using
         * the old VkImage; in this pipeline acquireSlot polls the slot
         * fence before reusing the slot, so by the time blitToGralloc
         * runs the previous frame on the same slot has already
         * drained. Other slots can still be in flight on different
         * images, but each slot rotates through its own command buffer
         * and only the recording slot can refer to this entry's
         * objects. */
        ALOGW("gralloc cache stale: handle=%p hash=%08x->%08x inode=%llu->%llu",
              anwb->handle, e.handleHash, curHash,
              (unsigned long long)e.dmabufInode,
              (unsigned long long)curInode);
        mDev.pfn()->DeviceWaitIdle(mDev.device());
        destroyEntry(e);
        mEntries.erase(it);
    }

    VkNativeBufferANDROID nbInfo = {};
    nbInfo.sType  = VK_STRUCTURE_TYPE_NATIVE_BUFFER_ANDROID;
    nbInfo.handle = anwb->handle;
    nbInfo.stride = anwb->stride;
    nbInfo.format = anwb->format;
    nbInfo.usage  = anwb->usage;

    VkImageCreateInfo ici = {};
    ici.sType         = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    ici.pNext         = &nbInfo;
    ici.imageType     = VK_IMAGE_TYPE_2D;
    ici.format        = VK_FORMAT_R8G8B8A8_UNORM;
    ici.extent.width  = width;
    ici.extent.height = height;
    ici.extent.depth  = 1;
    ici.mipLevels     = 1;
    ici.arrayLayers   = 1;
    ici.samples       = VK_SAMPLE_COUNT_1_BIT;
    ici.tiling        = VK_IMAGE_TILING_OPTIMAL;
    ici.usage         = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
    ici.sharingMode   = VK_SHARING_MODE_EXCLUSIVE;
    ici.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

    Entry entry = {};
    VkResult r = mDev.pfn()->CreateImage(mDev.device(), &ici, NULL, &entry.image);
    if (r != VK_SUCCESS) {
        ALOGE("gralloc vkCreateImage failed: %d", (int)r);
        return false;
    }

    VkImageViewCreateInfo vci = {};
    vci.sType    = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    vci.image    = entry.image;
    vci.viewType = VK_IMAGE_VIEW_TYPE_2D;
    vci.format   = VK_FORMAT_R8G8B8A8_UNORM;
    vci.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    vci.subresourceRange.levelCount = 1;
    vci.subresourceRange.layerCount = 1;
    r = mDev.pfn()->CreateImageView(mDev.device(), &vci, NULL, &entry.view);
    if (r != VK_SUCCESS) {
        ALOGE("gralloc vkCreateImageView failed: %d", (int)r);
        mDev.pfn()->DestroyImage(mDev.device(), entry.image, NULL);
        return false;
    }

    VkFramebufferCreateInfo fci = {};
    fci.sType           = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
    fci.renderPass      = mRenderPass;
    fci.attachmentCount = 1;
    fci.pAttachments    = &entry.view;
    fci.width           = width;
    fci.height          = height;
    fci.layers          = 1;
    r = mDev.pfn()->CreateFramebuffer(mDev.device(), &fci, NULL, &entry.framebuffer);
    if (r != VK_SUCCESS) {
        ALOGE("gralloc vkCreateFramebuffer failed: %d", (int)r);
        mDev.pfn()->DestroyImageView(mDev.device(), entry.view, NULL);
        mDev.pfn()->DestroyImage(mDev.device(), entry.image, NULL);
        return false;
    }

    entry.handleHash  = curHash;
    entry.dmabufInode = curInode;
    entry.lastUsed    = mFrame;
    auto res = mEntries.emplace(anwb->handle, entry);
    *outEntry = &res.first->second;
    return true;
}

}; /* namespace android */
