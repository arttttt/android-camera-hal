#define LOG_TAG "Cam-VulkanInputRing"
#include <utils/Log.h>

#include "VulkanInputRing.h"
#include "VulkanDeviceState.h"
#include "loader/VulkanPfn.h"

namespace android {

VulkanInputRing::VulkanInputRing(const VulkanDeviceState &dev)
    : mDev(dev), mSize(0) {
    for (int i = 0; i < kSlotCount; i++) {
        mBuf[i] = VK_NULL_HANDLE;
        mMem[i] = VK_NULL_HANDLE;
        mMap[i] = NULL;
    }
}

VulkanInputRing::~VulkanInputRing() { destroy(); }

void VulkanInputRing::destroy() {
    for (int i = 0; i < kSlotCount; i++) {
        if (mMap[i]) {
            mDev.pfn()->UnmapMemory(mDev.device(), mMem[i]);
            mMap[i] = NULL;
        }
        mDev.destroyBuffer(mBuf[i], mMem[i]);
        mBuf[i] = VK_NULL_HANDLE;
        mMem[i] = VK_NULL_HANDLE;
    }
    mSize = 0;
}

bool VulkanInputRing::ensureSize(size_t bytesPerSlot) {
    if (mSize == bytesPerSlot && mBuf[0] != VK_NULL_HANDLE)
        return true;

    destroy();

    for (int i = 0; i < kSlotCount; i++) {
        if (!mDev.createBuffer(&mBuf[i], &mMem[i], bytesPerSlot,
                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                /*exportable=*/ true)) {
            ALOGE("VulkanInputRing: createBuffer failed at slot %d", i);
            destroy();
            return false;
        }
        mDev.pfn()->MapMemory(mDev.device(), mMem[i], 0, bytesPerSlot, 0, &mMap[i]);
    }
    mSize = bytesPerSlot;
    return true;
}

VkBuffer VulkanInputRing::buffer(int idx) const {
    if (idx < 0 || idx >= kSlotCount) return VK_NULL_HANDLE;
    return mBuf[idx];
}

void *VulkanInputRing::mapped(int idx) const {
    if (idx < 0 || idx >= kSlotCount) return NULL;
    return mMap[idx];
}

void VulkanInputRing::invalidateFromGpu(int idx) {
    if (idx < 0 || idx >= kSlotCount) return;
    if (mMem[idx] == VK_NULL_HANDLE) return;

    VkMappedMemoryRange r = {};
    r.sType  = VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE;
    r.memory = mMem[idx];
    r.size   = VK_WHOLE_SIZE;
    mDev.pfn()->InvalidateMappedMemoryRanges(mDev.device(), 1, &r);
}

int VulkanInputRing::exportFd(int idx) {
    if (idx < 0 || idx >= kSlotCount) return -1;
    if (!mDev.isReady() || !mDev.pfn()->GetMemoryFdKHR) return -1;
    if (mMem[idx] == VK_NULL_HANDLE) return -1;

    VkMemoryGetFdInfoKHR gfi = {};
    gfi.sType = VK_STRUCTURE_TYPE_MEMORY_GET_FD_INFO_KHR;
    gfi.memory = mMem[idx];
    gfi.handleType = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT_KHR;

    int fd = -1;
    VkResult r = mDev.pfn()->GetMemoryFdKHR(mDev.device(), &gfi, &fd);
    if (r != VK_SUCCESS) {
        ALOGE("vkGetMemoryFdKHR failed for slot %d: %d", idx, r);
        return -1;
    }
    return fd;
}

}; /* namespace android */
