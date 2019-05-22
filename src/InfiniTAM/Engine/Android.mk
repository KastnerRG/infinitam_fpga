LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE    := Engine
LOCAL_SRC_FILES := ImageSourceEngine.cpp IMUSourceEngine.cpp OpenNIEngine.cpp
LOCAL_CFLAGS := -Werror -std=c++11
# -DCOMPILE_WITHOUT_CUDA
LOCAL_C_INCLUDES += $(CUDA_TOOLKIT_ROOT)/targets/armv7-linux-androideabi/include
LOCAL_C_INCLUDES += $(OPENNI2_ROOT)/Include
#LOCAL_LDLIBS := -landroid -lGLESv2

ifeq ($(COMPILE_FOR_TANGO),1)
LOCAL_SRC_FILES += TangoEngine.cpp
LOCAL_SHARED_LIBRARIES += tango_client_api tango_support_api
LOCAL_CFLAGS += -DCOMPILE_FOR_TANGO -DNO_DETAILED_PROFILING
endif

ifneq ($(COMPILE_WITH_OPENNI),1)
LOCAL_CFLAGS += -DCOMPILE_WITHOUT_OpenNI
endif

include $(BUILD_STATIC_LIBRARY)

