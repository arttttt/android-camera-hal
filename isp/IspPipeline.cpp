#define LOG_TAG "Cam-IspFactory"

#include "IspPipeline.h"
#include "VulkanIspPipeline.h"

#include <utils/Log.h>

namespace android {

IspPipeline *createIspPipeline() {
    ALOGD("ISP backend: vulkan");
    return new VulkanIspPipeline();
}

} /* namespace android */
