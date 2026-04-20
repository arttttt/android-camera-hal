LOCAL_PATH := $(call my-dir)

#-----------------------------------------------------------------------------
# Camera HAL module
#-----------------------------------------------------------------------------

include $(CLEAR_VARS)

LOCAL_MODULE := camera.$(TARGET_BOARD_PLATFORM)
LOCAL_MODULE_RELATIVE_PATH := hw

LOCAL_CFLAGS += -std=c++11
LOCAL_CFLAGS += -fno-short-enums
LOCAL_CFLAGS += -Wno-unused-parameter -Wno-missing-field-initializers
LOCAL_CFLAGS += -pthread

LOCAL_CFLAGS += -DV4L2DEVICE_BUF_COUNT=4

# Configure and open device once on HAL start
LOCAL_CFLAGS += -DV4L2DEVICE_OPEN_ONCE


# Compile debug code - comment out to disable
#LOCAL_CFLAGS += -UNDEBUG -DDEBUG


LOCAL_STATIC_LIBRARIES := \
    libyuv_static

LOCAL_SHARED_LIBRARIES := \
    liblog \
    libutils \
    libcutils \
    libcamera_client \
    libui \
    libjpeg \
    libcamera_metadata \
    libskia \
    libandroid_runtime

LOCAL_C_INCLUDES += \
    external/jpeg \
    external/libyuv/files/include \
    frameworks/native/include/media/hardware \
    $(call include-path-for, camera)

LOCAL_C_INCLUDES += \
    external/skia/include/core/ \
    frameworks/base/core/jni/android/graphics \
    frameworks/native/include \
    prebuilts/ndk/current/platforms/android-24/arch-arm/usr/include

# Module-internal include dirs
LOCAL_C_INCLUDES += \
    $(LOCAL_PATH)/hal \
    $(LOCAL_PATH)/v4l2 \
    $(LOCAL_PATH)/isp \
    $(LOCAL_PATH)/isp/vulkan \
    $(LOCAL_PATH)/isp/vulkan/encode \
    $(LOCAL_PATH)/util

LOCAL_SRC_FILES += \
    hal/HalModule.cpp \
    hal/Camera.cpp \
    hal/metadata/CameraStaticMetadata.cpp \
    hal/metadata/RequestTemplateBuilder.cpp \
    hal/metadata/ResultMetadataBuilder.cpp \
    hal/3a/AutoFocusController.cpp \
    hal/3a/ExposureControl.cpp \
    hal/jpeg/JpegEncoder.cpp \
    hal/pipeline/BufferProcessor.cpp \
    hal/pipeline/StreamConfig.cpp \
    v4l2/V4l2Device.cpp \
    isp/IspPipeline.cpp \
    isp/IspParams.cpp \
    isp/sensor/IspCalibration.cpp \
    isp/vulkan/VulkanIspPipeline.cpp \
    isp/vulkan/runtime/VulkanDeviceState.cpp \
    isp/vulkan/runtime/loader/VulkanLoader.cpp \
    isp/vulkan/runtime/loader/HalHmiVulkanLoader.cpp \
    isp/vulkan/runtime/loader/SystemVulkanLoader.cpp \
    isp/vulkan/io/VulkanInputRing.cpp \
    isp/vulkan/io/VulkanGrallocCache.cpp \
    isp/vulkan/encode/VulkanYuvEncoder.cpp

LOCAL_SHARED_LIBRARIES += libvulkan

include $(BUILD_SHARED_LIBRARY)

