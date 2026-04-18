#define LOG_TAG "Cam-IspFactory"
#include <utils/Log.h>
#include <cstring>
#include <cutils/properties.h>

#include "IspPipeline.h"
#include "VulkanIspPipeline.h"
#include "HwIspPipeline.h"

namespace android {

IspPipeline *createIspPipeline() {
    char backend[PROPERTY_VALUE_MAX] = {0};
    property_get("persist.camera.isp_backend", backend, "vulkan");

    ALOGD("ISP backend: %s", backend);
    if (!strcmp(backend, "hwisp"))
        return new HwIspPipeline();
    return new VulkanIspPipeline();
}

} /* namespace android */
