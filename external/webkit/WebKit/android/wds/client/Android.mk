LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)

LOCAL_SRC_FILES := \
	AdbConnection.cpp \
	ClientUtils.cpp \
	Device.cpp \
	main.cpp

LOCAL_STATIC_LIBRARIES := liblog libutils libcutils

LOCAL_MODULE:= wdsclient

include $(BUILD_HOST_EXECUTABLE)
