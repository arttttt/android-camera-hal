#include <cutils/properties.h>

#include "VulkanLoader.h"
#include "HalHmiVulkanLoader.h"
#include "SystemVulkanLoader.h"

namespace android {

/* First API level where the standard Android Vulkan loader exposes
 * AHardwareBuffer integration (VK_ANDROID_external_memory_android_
 * hardware_buffer) natively — no HAL-layer bypass needed. */
static const int API_OREO = 26;

VulkanLoader *createVulkanLoader() {
    int sdk = property_get_int32("ro.build.version.sdk", 0);
    if (sdk >= API_OREO)
        return new SystemVulkanLoader();
    return new HalHmiVulkanLoader();
}

} /* namespace android */
