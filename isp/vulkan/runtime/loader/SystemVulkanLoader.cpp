#define LOG_TAG "Cam-VkLoader"
#include <utils/Log.h>
#include <cutils/properties.h>

#include "SystemVulkanLoader.h"

namespace android {

bool SystemVulkanLoader::load() {
    int sdk = property_get_int32("ro.build.version.sdk", 0);
    ALOGE("SystemVulkanLoader not implemented — device reports Android API %d, "
          "which should use the standard libvulkan.so path "
          "(VK_ANDROID_external_memory_android_hardware_buffer etc.), "
          "but that integration has not been written yet. "
          "Current project target is Android 7.1.2 / API 25 only, "
          "handled by HalHmiVulkanLoader.", sdk);
    return false;
}

} /* namespace android */
