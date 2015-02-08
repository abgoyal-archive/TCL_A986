LOCAL_PATH:=$(call my-dir)

# iwlib

include $(CLEAR_VARS)

LOCAL_SRC_FILES := iwlib.c
LOCAL_CFLAGS += -Wstrict-prototypes -Wmissing-prototypes -Wshadow -Wpointer-arith -Wcast-qual -Winline -MMD -fPIC
LOCAL_STATIC_LIBRARIES := libcutils libc libm
LOCAL_MODULE := libiw
include $(BUILD_STATIC_LIBRARY)

# iwconfig

include $(CLEAR_VARS)

LOCAL_SRC_FILES := iwconfig.c
LOCAL_CFLAGS += -Wstrict-prototypes -Wmissing-prototypes -Wshadow -Wpointer-arith -Wcast-qual -Winline -MMD -fPIC
LOCAL_STATIC_LIBRARIES := libcutils libc libm libiw
#LOCAL_FORCE_STATIC_EXECUTABLE := true
LOCAL_MODULE_PATH := $(TARGET_OUT_OPTIONAL_EXECUTABLES)  # install to system/xbin
#LOCAL_UNSTRIPPED_PATH := $(TARGET_ROOT_OUT_UNSTRIPPED)
#LOCAL_MODULE_TAGS := eng user
LOCAL_MODULE:= iwconfig
include $(BUILD_EXECUTABLE)

# iwevent

include $(CLEAR_VARS)

LOCAL_SRC_FILES := iwevent.c
LOCAL_CFLAGS += -Wstrict-prototypes -Wmissing-prototypes -Wshadow -Wpointer-arith -Wcast-qual -Winline -MMD -fPIC
LOCAL_STATIC_LIBRARIES := libcutils libc libm libiw
#LOCAL_FORCE_STATIC_EXECUTABLE := true
LOCAL_MODULE_PATH := $(TARGET_OUT_OPTIONAL_EXECUTABLES)  # install to system/xbin
#LOCAL_UNSTRIPPED_PATH := $(TARGET_ROOT_OUT_UNSTRIPPED)
#LOCAL_MODULE_TAGS := eng user
LOCAL_MODULE:= iwevent
include $(BUILD_EXECUTABLE)

# iwgetid

include $(CLEAR_VARS)

LOCAL_SRC_FILES := iwgetid.c
LOCAL_CFLAGS += -Wstrict-prototypes -Wmissing-prototypes -Wshadow -Wpointer-arith -Wcast-qual -Winline -MMD -fPIC
LOCAL_STATIC_LIBRARIES := libcutils libc libm libiw
#LOCAL_FORCE_STATIC_EXECUTABLE := true
LOCAL_MODULE_PATH := $(TARGET_OUT_OPTIONAL_EXECUTABLES)  # install to system/xbin
#LOCAL_UNSTRIPPED_PATH := $(TARGET_ROOT_OUT_UNSTRIPPED)
#LOCAL_MODULE_TAGS := eng user
LOCAL_MODULE:= iwgetid
include $(BUILD_EXECUTABLE)

# iwlist

include $(CLEAR_VARS)

LOCAL_SRC_FILES := iwlist.c
LOCAL_CFLAGS += -Wstrict-prototypes -Wmissing-prototypes -Wshadow -Wpointer-arith -Wcast-qual -Winline -MMD -fPIC
LOCAL_STATIC_LIBRARIES := libcutils libc libm libiw
#LOCAL_FORCE_STATIC_EXECUTABLE := true
LOCAL_MODULE_PATH := $(TARGET_OUT_OPTIONAL_EXECUTABLES)  # install to system/xbin
#LOCAL_UNSTRIPPED_PATH := $(TARGET_ROOT_OUT_UNSTRIPPED)
#LOCAL_MODULE_TAGS := eng user
LOCAL_MODULE:= iwlist
include $(BUILD_EXECUTABLE)

# iwpriv

include $(CLEAR_VARS)

LOCAL_SRC_FILES := iwpriv.c
LOCAL_CFLAGS += -Wstrict-prototypes -Wmissing-prototypes -Wshadow -Wpointer-arith -Wcast-qual -Winline -MMD -fPIC
LOCAL_STATIC_LIBRARIES := libcutils libc libm libiw
#LOCAL_FORCE_STATIC_EXECUTABLE := true
LOCAL_MODULE_PATH := $(TARGET_OUT_OPTIONAL_EXECUTABLES)  # install to system/xbin
#LOCAL_UNSTRIPPED_PATH := $(TARGET_ROOT_OUT_UNSTRIPPED)
#LOCAL_MODULE_TAGS := eng user
LOCAL_MODULE:= iwpriv
include $(BUILD_EXECUTABLE)

# iwspy

include $(CLEAR_VARS)

LOCAL_SRC_FILES := iwspy.c
LOCAL_CFLAGS += -Wstrict-prototypes -Wmissing-prototypes -Wshadow -Wpointer-arith -Wcast-qual -Winline -MMD -fPIC
LOCAL_STATIC_LIBRARIES := libcutils libc libm libiw
#LOCAL_FORCE_STATIC_EXECUTABLE := true
LOCAL_MODULE_PATH := $(TARGET_OUT_OPTIONAL_EXECUTABLES)  # install to system/xbin
#LOCAL_UNSTRIPPED_PATH := $(TARGET_ROOT_OUT_UNSTRIPPED)
#LOCAL_MODULE_TAGS := eng user
LOCAL_MODULE:= iwspy
include $(BUILD_EXECUTABLE)

