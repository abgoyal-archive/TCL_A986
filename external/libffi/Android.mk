LOCAL_PATH:= $(call my-dir)


#
# Build rules for the target.
#

# We only build ffi at all for non-arm, non-x86 targets.
ifneq ($(TARGET_ARCH),arm)
    ifneq ($(TARGET_ARCH),x86)

       include $(CLEAR_VARS)

       ffi_arch := $(TARGET_ARCH)
       ffi_os := $(TARGET_OS)

       # This include just keeps the nesting a bit saner.
       include $(LOCAL_PATH)/Libffi.mk

       LOCAL_MODULE := libffi

       include $(BUILD_SHARED_LIBRARY)

    endif
endif

# Also include the rules for the test suite.
include external/libffi/testsuite/Android.mk

