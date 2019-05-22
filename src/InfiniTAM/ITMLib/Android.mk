LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

LOCAL_ARM_NEON := true

GCC?=$(NDK_ROOT)/toolchains/arm-linux-androideabi-4.6/gen_standalone/linux-x86_64/bin/arm-linux-androideabi-g++
NVCC?=$(CUDA_TOOLKIT_ROOT)/bin/nvcc -ccbin $(GCC) -target-cpu-arch=ARM -m32 -arch=sm_30 -O3 -Xptxas '-dlcm=ca' -target-os-variant=Android --use_fast_math

ITMLIB_PATH?=.

# Needs to be equal to -DCOMPILE_FOR_TANGO on Tango device
TANGO_FLAGS ?=

MY_FILE_LIST := $(wildcard $(ITMLIB_PATH)/Engine/*.cpp) $(wildcard $(ITMLIB_PATH)/Engine/DeviceSpecific/CPU/*.cpp) $(wildcard $(ITMLIB_PATH)/Utils/*.cpp) $(wildcard $(ITMLIB_PATH)/Objects/*.cpp) $(wildcard $(ITMLIB_PATH)/Engine/DeviceSpecific/CUDA/*.cu)
MY_OBJ_LIST := $(MY_FILE_LIST:%.cu=%.o)
MY_OBJ_LIST := $(MY_OBJ_LIST:%.cpp=%.o)
MY_MODULE := libITMLib.a
MY_INCLUDES += $(CUDA_TOOLKIT_ROOT)/targets/armv7-linux-androideabi/include

DEPEND_PATH := $(ITMLIB_PATH)/.depend


# Dependencies building, only for Makefile (not for Android.mk)
# Note / TODO the dependencies are still not enough, maybe implement the TODO below will suffice?
#
ifneq ($(strip $(MY_FILE_LIST)),)

depend: $(DEPEND_PATH)

$(DEPEND_PATH):
	rm -f "$(DEPEND_PATH)"
	#$(GCC) -MM -MT $(TANGO_FLAGS) $(MY_OBJ_LIST) $(MY_FILE_LIST) > "$(DEPEND_PATH)";

-include $(DEPEND_PATH)
endif

# TODO make a variable NVCC_FLAGS so we can do nvcc -M -MT
#$(GCC) -MM -MT "$@" "$<" >> "$(DEPEND_PATH)";
%.o: %.cu
	$(NVCC) $(CFLAGS) $(EXTRA_CFLAGS) $(TANGO_FLAGS) -c -o "$@" "$<"


%.o: %.cpp
	$(GCC) $(TANGO_FLAGS) -MM -MT "$@" "$<" >> "$(DEPEND_PATH)";
	$(GCC) $(TANGO_FLAGS) -O3 -march=armv7-a -mtune=cortex-a15 -c -o "$@" "$<" $(MY_INCLUDES:%=-I%)


$(MY_MODULE): $(MY_OBJ_LIST)
	$(NVCC) -lib -o $(ITMLIB_PATH)/"$@" $^

LOCAL_MODULE    := libITMLib
LOCAL_SRC_FILES := $(MY_MODULE)

include $(PREBUILT_STATIC_LIBRARY)

