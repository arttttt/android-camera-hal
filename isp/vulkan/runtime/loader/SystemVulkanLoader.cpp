#define LOG_TAG "Cam-VkLoader"
#include <utils/Log.h>
#include <dlfcn.h>

#include "SystemVulkanLoader.h"

namespace android {

SystemVulkanLoader::SystemVulkanLoader()
    : mDso(nullptr),
      mGetInstanceProcAddr(nullptr),
      mCreateInstance(nullptr),
      mEnumerateInstanceExtensionProperties(nullptr) {}

SystemVulkanLoader::~SystemVulkanLoader() {
    /* The dispatch table handed to the caller points into this library, so
     * the handle is dropped only together with the loader itself. */
    if (mDso) { dlclose(mDso); mDso = nullptr; }
}

bool SystemVulkanLoader::load() {
    /* By soname, not by path: the linker namespace resolves it to the copy
     * that matches this process, which is what lets a vendor process reach
     * the system loader at all. An absolute path would pin one ABI. */
    mDso = dlopen("libvulkan.so", RTLD_NOW | RTLD_LOCAL);
    if (!mDso) {
        ALOGE("System: dlopen libvulkan.so failed: %s", dlerror());
        return false;
    }

    mGetInstanceProcAddr = (PFN_vkGetInstanceProcAddr)
        dlsym(mDso, "vkGetInstanceProcAddr");
    if (!mGetInstanceProcAddr) {
        ALOGE("System: dlsym vkGetInstanceProcAddr failed: %s", dlerror());
        return false;
    }

    /* The other two bootstrap entry points are resolved through
     * vkGetInstanceProcAddr with a null instance, the form the specification
     * requires every loader to support, rather than by another dlsym. That
     * keeps layer interposition intact: a layer chain is free to return its
     * own vkCreateInstance here, and dlsym would step around it. */
    mCreateInstance = (PFN_vkCreateInstance)
        mGetInstanceProcAddr(VK_NULL_HANDLE, "vkCreateInstance");
    mEnumerateInstanceExtensionProperties =
        (PFN_vkEnumerateInstanceExtensionProperties)
        mGetInstanceProcAddr(VK_NULL_HANDLE,
                             "vkEnumerateInstanceExtensionProperties");

    if (!mCreateInstance || !mEnumerateInstanceExtensionProperties) {
        ALOGE("System: libvulkan.so lacks bootstrap entry points "
              "(CreateInstance=%p, EnumerateInstanceExtensionProperties=%p)",
              (void *) mCreateInstance,
              (void *) mEnumerateInstanceExtensionProperties);
        return false;
    }

    ALOGD("System: loaded libvulkan.so (GIPA=%p, CreateInstance=%p)",
          (void *) mGetInstanceProcAddr, (void *) mCreateInstance);
    return true;
}

} /* namespace android */
