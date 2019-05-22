LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_MODULE    := ORUtils
LOCAL_SRC_FILES := Dummy.cpp OpenCLContext.cpp
LOCAL_CFLAGS := -Werror -std=c++11
LOCAL_C_INCLUDES += $(CUDA_TOOLKIT_ROOT)/targets/armv7-linux-androideabi/include
LOCAL_C_INCLUDES +=

ifeq ($(COMPILE_FOR_TANGO),1)
LOCAL_CFLAGS += -DCOMPILE_FOR_TANGO -DNO_DETAILED_PROFILING
endif

ifneq ($(COMPILE_WITH_OPENNI),1)
LOCAL_CFLAGS += -DCOMPILE_WITHOUT_OpenNI
endif

include $(BUILD_STATIC_LIBRARY)

