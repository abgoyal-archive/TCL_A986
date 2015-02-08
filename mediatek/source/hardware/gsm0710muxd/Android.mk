LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

LOCAL_MODULE:= gsm0710muxd

LOCAL_SRC_FILES:= \
	src/gsm0710muxd.c \
	src/gsm0710muxd_fc.c \

LOCAL_SHARED_LIBRARIES := \
	libcutils \

LOCAL_CFLAGS := \
	-DMUX_ANDROID \
	-D__CCMNI_SUPPORT__ \
	-D__MUXD_FLOWCONTROL__ \

ifeq ($(GEMINI),yes)
	LOCAL_CFLAGS += -DMTK_GEMINI
else
    ifneq ($(MTK_SHARE_MODEM_SUPPORT),1)
   LOCAL_CFLAGS += -DMTK_GEMINI 
  endif
endif

ifneq ($(MTK_INTERNEL),yes)
	LOCAL_CFLAGS += -D__PRODUCTION_RELEASE__
endif

ifeq ($(MTK_VT3G324M_SUPPORT),yes)
  LOCAL_CFLAGS += -D__ANDROID_VT_SUPPORT__
endif

LOCAL_C_INCLUDES := $(KERNEL_HEADERS)
LOCAL_LDLIBS := -lpthread

include $(BUILD_EXECUTABLE)

