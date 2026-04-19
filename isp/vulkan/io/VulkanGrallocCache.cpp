#define LOG_TAG "Cam-VulkanGrallocCache"
#include <utils/Log.h>
#include <system/window.h>

#include "VulkanGrallocCache.h"
#include "runtime/VulkanDeviceState.h"
#include "runtime/loader/VulkanPfn.h"

/* VK_ANDROID_native_buffer type — absent from the android-24 NDK vulkan.h
 * but exposed by the HAL-variant loader. */
#ifndef VK_ANDROID_NATIVE_BUFFER_NUMBER
#define VK_ANDROID_NATIVE_BUFFER_NUMBER 11
#define VK_STRUCTURE_TYPE_NATIVE_BUFFER_ANDROID ((VkStructureType)1000010000)
typedef struct VkNativeBufferANDROID {
    VkStructureType sType;
    const void *pNext;
    const struct native_handle *handle;
    int stride;
    int format;
    int usage;
} VkNativeBufferANDROID;
#endif

namespace android {

VulkanGrallocCache::VulkanGrallocCache(const VulkanDeviceState &dev)
    : mDev(dev)
    , mRenderPass(VK_NULL_HANDLE) {}

VulkanGrallocCache::~VulkanGrallocCache() {
    clear();
}

void VulkanGrallocCache::setRenderPass(VkRenderPass rp) {
    mRenderPass = rp;
}

void VulkanGrallocCache::clear() {
    if (!mDev.isReady() || mDev.device() == VK_NULL_HANDLE) {
        mEntries.clear();
        return;
    }
    for (auto &kv : mEntries) {
        if (kv.second.framebuffer) {
            mDev.pfn()->DestroyFramebuffer(mDev.device(), kv.second.framebuffer, NULL);
        }
        if (kv.second.view) {
            mDev.pfn()->DestroyImageView(mDev.device(), kv.second.view, NULL);
        }
        if (kv.second.image) {
            mDev.pfn()->DestroyImage(mDev.device(), kv.second.image, NULL);
        }
    }
    mEntries.clear();
}

bool VulkanGrallocCache::getOrCreate(ANativeWindowBuffer *anwb,
                                      unsigned width, unsigned height,
                                      Entry **outEntry) {
    auto it = mEntries.find(anwb->handle);
    if (it != mEntries.end()) {
        *outEntry = &it->second;
        return true;
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

    entry.layoutReady = false;
    auto res = mEntries.emplace(anwb->handle, entry);
    *outEntry = &res.first->second;
    ALOGD("gralloc image cached: handle=%p image=%p view=%p (size=%zu)",
          anwb->handle, entry.image, entry.view, mEntries.size());
    return true;
}

}; /* namespace android */
