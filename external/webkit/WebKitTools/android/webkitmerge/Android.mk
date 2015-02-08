LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

LOCAL_SRC_FILES := webkitmerge.cpp

LOCAL_MODULE := webkitmerge

LOCAL_MODULE_TAGS := eng

include $(BUILD_HOST_EXECUTABLE)
