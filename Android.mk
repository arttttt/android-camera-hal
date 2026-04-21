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
    libyuv_static \
    libjsoncpp

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
    external/jsoncpp/include \
    frameworks/native/include/media/hardware \
    $(call include-path-for, camera)

LOCAL_C_INCLUDES += \
    external/skia/include/core/ \
    frameworks/base/core/jni/android/graphics \
    frameworks/native/include \
    prebuilts/ndk/current/platforms/android-24/arch-arm/usr/include

# Module-internal include dirs
LOCAL_C_INCLUDES += \
    $(LOCAL_PATH)/base \
    $(LOCAL_PATH)/hal \
    $(LOCAL_PATH)/hal/pipeline \
    $(LOCAL_PATH)/hal/pipeline/stages \
    $(LOCAL_PATH)/v4l2 \
    $(LOCAL_PATH)/isp \
    $(LOCAL_PATH)/isp/vulkan \
    $(LOCAL_PATH)/isp/vulkan/encode \
    $(LOCAL_PATH)/util

LOCAL_SRC_FILES += \
    base/EventFd.cpp \
    base/ThreadBase.cpp \
    hal/HalModule.cpp \
    hal/Camera.cpp \
    hal/metadata/CameraStaticMetadata.cpp \
    hal/metadata/RequestTemplateBuilder.cpp \
    hal/metadata/ResultMetadataBuilder.cpp \
    hal/3a/AutoFocusController.cpp \
    hal/3a/ExposureControl.cpp \
    hal/jpeg/JpegEncoder.cpp \
    hal/pipeline/BufferProcessor.cpp \
    hal/pipeline/InFlightTracker.cpp \
    hal/pipeline/Pipeline.cpp \
    hal/pipeline/PipelineThread.cpp \
    hal/pipeline/RequestThread.cpp \
    hal/pipeline/StreamConfig.cpp \
    hal/pipeline/stages/ApplySettingsStage.cpp \
    hal/pipeline/stages/CaptureStage.cpp \
    hal/pipeline/stages/DemosaicBlitStage.cpp \
    hal/pipeline/stages/ResultDispatchStage.cpp \
    hal/pipeline/stages/ShutterNotifyStage.cpp \
    v4l2/V4l2CaptureThread.cpp \
    v4l2/V4l2Device.cpp \
    v4l2/V4l2Source.cpp \
    isp/IspPipeline.cpp \
    isp/IspParams.cpp \
    isp/sensor/SensorTuning.cpp \
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

#-----------------------------------------------------------------------------
# Per-sensor tuning profiles (Treble-compatible /vendor path)
#-----------------------------------------------------------------------------

include $(CLEAR_VARS)
LOCAL_MODULE       := imx179_primax.json
LOCAL_MODULE_CLASS := ETC
LOCAL_MODULE_TAGS  := optional
LOCAL_SRC_FILES    := tuning/imx179_primax.json
LOCAL_MODULE_PATH  := $(TARGET_OUT_VENDOR_ETC)/camera/tuning
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_MODULE       := ov5693_sunny.json
LOCAL_MODULE_CLASS := ETC
LOCAL_MODULE_TAGS  := optional
LOCAL_SRC_FILES    := tuning/ov5693_sunny.json
LOCAL_MODULE_PATH  := $(TARGET_OUT_VENDOR_ETC)/camera/tuning
include $(BUILD_PREBUILT)

