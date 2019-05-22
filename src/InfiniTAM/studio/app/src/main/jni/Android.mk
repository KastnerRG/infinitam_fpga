LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := libcudart_static
LOCAL_SRC_FILES  := $(CUDA_TOOLKIT_ROOT)/targets/armv7-linux-androideabi/lib/libcudart_static.a 
LOCAL_EXPORT_C_INCLUDES := $(CUDA_TOOLKIT_ROOT)/targets/armv7-linux-androideabi/include
include $(PREBUILT_STATIC_LIBRARY)


COMPILE_WITH_OPENNI := 0
#OPENNI2_ROOT:=/local/olaf/compile/oni2/OpenNI2_x86_64/
#OPENNI2_ROOT:=/home/qkgautier/workspace/OpenNI2
OPENNI2_ROOT:=
ifneq ($(OPENNI2_ROOT),)

MY_OPENNI2_LIBDIR := $(OPENNI2_ROOT)/Packaging/OpenNI-android-2.2/
MY_OPENNI_MODULE := OpenNI2 OniFile PSLink PS1080 usb

include $(CLEAR_VARS)
LOCAL_MODULE := OpenNI2
LOCAL_SRC_FILES := $(MY_OPENNI2_LIBDIR)libOpenNI2.so
LOCAL_EXPORT_C_INCLUDES := $(OPENNI2_ROOT)/Include
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := OniFile
LOCAL_SRC_FILES := $(MY_OPENNI2_LIBDIR)libOniFile.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := PS1080
LOCAL_SRC_FILES := $(MY_OPENNI2_LIBDIR)libPS1080.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := PSLink
LOCAL_SRC_FILES := $(MY_OPENNI2_LIBDIR)libPSLink.so
include $(PREBUILT_SHARED_LIBRARY)

include $(CLEAR_VARS)
LOCAL_MODULE := usb
LOCAL_SRC_FILES := $(MY_OPENNI2_LIBDIR)libusb.so
include $(PREBUILT_SHARED_LIBRARY)

COMPILE_WITH_OPENNI = 1

endif



COMPILE_FOR_TANGO := 1





include $(CLEAR_VARS)




LOCAL_MODULE    := InfiniTAM
LOCAL_SRC_FILES := InfiniTAMApp.cpp jniExport.cpp JavaMethods.cpp TouchHandler.cpp
LOCAL_CFLAGS := -std=c++11 -Werror #-DCOMPILE_WITHOUT_CUDA
LOCAL_LDLIBS := -landroid -lGLESv1_CM -llog
LOCAL_SHARED_LIBRARIES := $(MY_OPENNI_MODULE)
LOCAL_STATIC_LIBRARIES := Engine Utils ITMLib ORUtils cudart_static


ifeq ($(COMPILE_FOR_TANGO),1)
LOCAL_SRC_FILES += tangoMain.cpp jniTango.cpp
LOCAL_CFLAGS += -DCOMPILE_FOR_TANGO -DNO_DETAILED_PROFILING
LOCAL_SHARED_LIBRARIES += opencv_java
endif


ifneq ($(COMPILE_WITH_OPENNI),1)
LOCAL_CFLAGS += -DCOMPILE_WITHOUT_OpenNI
endif

include $(BUILD_SHARED_LIBRARY)


$(call import-add-path, $(LOCAL_PATH))

include ../../../../../Engine/Android.mk
include ../../../../../Utils/Android.mk
include ../../../../../ITMLib/Android.mk
include ../../../../../ORUtils/Android.mk


ifeq ($(COMPILE_FOR_TANGO),1)
$(call import-module,tango_client_api)
$(call import-module,tango_support_api)
endif
