ifeq ($(MTK_WAPI_SUPPORT), yes)
LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)
LOCAL_PREBUILT_LIBS := libwapi.so
include $(BUILD_MULTI_PREBUILT)
endif
