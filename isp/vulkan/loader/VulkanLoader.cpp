#include "VulkanLoader.h"
#include "HalHmiVulkanLoader.h"

namespace android {

VulkanLoader *createVulkanLoader() {
    /* Project target is Android 7.1.2 / Tegra K1 — always HMI bypass.
     *
     * When Android 8+ support is added, switch here based on:
     *   property_get_int32("ro.build.version.sdk", 0) >= 26
     * (returning SystemVulkanLoader for newer, HalHmiVulkanLoader for
     * older). #include "SystemVulkanLoader.h" at that point. */
    return new HalHmiVulkanLoader();
}

} /* namespace android */
