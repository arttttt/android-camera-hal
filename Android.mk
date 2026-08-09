LOCAL_PATH := $(call my-dir)

#-----------------------------------------------------------------------------
# Camera HAL module
#-----------------------------------------------------------------------------

include $(CLEAR_VARS)

LOCAL_MODULE := camera.$(TARGET_BOARD_PLATFORM)
LOCAL_MODULE_RELATIVE_PATH := hw

# Install to /vendor/lib/hw. This is a hardware module: since O it belongs on
# the vendor side, next to the tuning profiles below and to the HIDL provider
# that dlopens it, which is itself a vendor binary.
LOCAL_PROPRIETARY_MODULE := true

LOCAL_CFLAGS += -std=c++14
LOCAL_CFLAGS += -fno-short-enums
LOCAL_CFLAGS += -Wno-unused-parameter -Wno-missing-field-initializers
LOCAL_CFLAGS += -pthread

LOCAL_CFLAGS += -DV4L2DEVICE_BUF_COUNT=8


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
    libcamera_metadata

LOCAL_C_INCLUDES += \
    external/jpeg \
    external/libyuv/files/include \
    external/jsoncpp/include \
    frameworks/native/include/media/hardware \
    $(call include-path-for, camera)

LOCAL_C_INCLUDES += \
    frameworks/native/include \
    frameworks/native/vulkan/include \
    prebuilts/ndk/current/platforms/android-24/arch-arm/usr/include

# ANativeWindow and the gralloc usage bits: system/window.h sits in
# system/core/include on the oldest branch we build against and moved into
# the nativewindow library on the newer ones. Both paths are listed rather
# than picked with a conditional -- an include directory that does not exist
# is ignored, so one line covers every branch. It used to arrive by accident
# through libandroid_runtime's exported headers, which this module no longer
# links.
LOCAL_C_INCLUDES += frameworks/native/libs/nativewindow/include

# Module-internal include dirs
LOCAL_C_INCLUDES += \
    $(LOCAL_PATH)/base \
    $(LOCAL_PATH)/hal \
    $(LOCAL_PATH)/hal/ipa \
    $(LOCAL_PATH)/hal/pipeline \
    $(LOCAL_PATH)/hal/pipeline/stages \
    $(LOCAL_PATH)/v4l2 \
    $(LOCAL_PATH)/isp \
    $(LOCAL_PATH)/isp/nvblit \
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
    hal/3a/AutoExposureController.cpp \
    hal/3a/AutoFocusController.cpp \
    hal/3a/AwbFactory.cpp \
    hal/3a/BayesianAwbController.cpp \
    hal/3a/GrayWorldAwbController.cpp \
    hal/ipa/Ipa3A.cpp \
    hal/ipa/NeonStatsEncoder.cpp \
    hal/ipa/StatsWorker.cpp \
    hal/ipa/StubIpa.cpp \
    hal/jpeg/JpegEncoder.cpp \
    hal/pipeline/BufferProcessor.cpp \
    hal/pipeline/InFlightTracker.cpp \
    hal/pipeline/Pipeline.cpp \
    hal/pipeline/PipelineThread.cpp \
    hal/pipeline/RequestThread.cpp \
    hal/pipeline/ResultThread.cpp \
    hal/pipeline/JpegWorker.cpp \
    hal/pipeline/StreamConfig.cpp \
    hal/pipeline/stages/ApplySettingsStage.cpp \
    hal/pipeline/stages/CaptureStage.cpp \
    hal/pipeline/stages/DemosaicBlitStage.cpp \
    hal/pipeline/stages/ResultDispatchStage.cpp \
    hal/pipeline/stages/ShutterNotifyStage.cpp \
    hal/pipeline/stages/StatsDispatchStage.cpp \
    hal/pipeline/stages/StatsProcessStage.cpp \
    v4l2/V4l2CaptureThread.cpp \
    v4l2/V4l2Device.cpp \
    v4l2/V4l2Source.cpp \
    isp/IspPipeline.cpp \
    isp/IspParams.cpp \
    isp/nvblit/NvBlitContext.cpp \
    isp/sensor/DelayedControls.cpp \
    isp/sensor/Pwl.cpp \
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

#-----------------------------------------------------------------------------
# Diagnostic standalone: libnvblit smoke test
#-----------------------------------------------------------------------------

include $(CLEAR_VARS)
LOCAL_MODULE      := nvblit_probe
LOCAL_MODULE_TAGS := optional
LOCAL_CFLAGS      += -std=c++14 -Wno-unused-parameter
LOCAL_SRC_FILES   := tools/nvblit_probe/nvblit_probe.cpp
LOCAL_SHARED_LIBRARIES := liblog libutils libcutils libui
LOCAL_C_INCLUDES  += frameworks/native/include
include $(BUILD_EXECUTABLE)

#-----------------------------------------------------------------------------
# Diagnostic standalone: STORAGE_BIT VkImage over gralloc import
#-----------------------------------------------------------------------------

include $(CLEAR_VARS)
LOCAL_MODULE      := vk_storage_probe
LOCAL_MODULE_TAGS := optional
LOCAL_CFLAGS      += -std=c++14 -Wno-unused-parameter
LOCAL_SRC_FILES   := tools/vk_storage_probe/vk_storage_probe.cpp
LOCAL_SHARED_LIBRARIES := liblog libutils libcutils libui libdl
LOCAL_C_INCLUDES  += \
    frameworks/native/include \
    frameworks/native/vulkan/include \
    prebuilts/ndk/current/platforms/android-24/arch-arm/usr/include
include $(BUILD_EXECUTABLE)

