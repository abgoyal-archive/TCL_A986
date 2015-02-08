LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

LOCAL_SRC_FILES := \
	main.cpp

# Pull the webkit definitions from the base webkit makefile.
LOCAL_SHARED_LIBRARIES := libwebcore $(WEBKIT_SHARED_LIBRARIES)
LOCAL_LDLIBS := $(WEBKIT_LDLIBS)

LOCAL_MODULE := webcore_test

LOCAL_MODULE_TAGS := optional

include $(BUILD_EXECUTABLE)
