LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

LOCAL_SRC_FILES := ip-up-vpn.c
LOCAL_SHARED_LIBRARIES := libcutils
LOCAL_MODULE := ip-up-vpn
LOCAL_MODULE_PATH := $(TARGET_OUT_ETC)/ppp

include $(BUILD_EXECUTABLE)
