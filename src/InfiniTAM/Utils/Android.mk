LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

# OpenCV
ifeq ($(COMPILE_FOR_TANGO),1)
INSTALL_CUDA_LIBRARIES:=on
OPENCV_INSTALL_MODULES:=on
CUDA_TOOLKIT_DIR=$(CUDA_TOOLKIT_ROOT)
include $(HOME)/NVPACK/OpenCV-2.4.8.2-Tegra-sdk/sdk/native/jni/OpenCV-tegra5-static-cuda.mk
endif

LOCAL_MODULE    := Utils
LOCAL_SRC_FILES := FileUtils.cpp
LOCAL_CFLAGS := -Werror -std=c++11
# -DCOMPILE_WITHOUT_CUDA
LOCAL_C_INCLUDES += $(CUDA_TOOLKIT_ROOT)/targets/armv7-linux-androideabi/include
#LOCAL_LDLIBS := -landroid -lGLESv2

ifeq ($(COMPILE_FOR_TANGO),1)
LOCAL_SRC_FILES += TangoImages.cpp DataWriter.cpp tinyply.cpp
LOCAL_SHARED_LIBRARIES += tango_client_api tango_support_api
LOCAL_CFLAGS += -DCOMPILE_FOR_TANGO
endif

include $(BUILD_STATIC_LIBRARY)

