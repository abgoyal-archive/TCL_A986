ifeq ($(DEFAULT_GOAL),droid)
    LOCAL_PATH:= $(call my-dir)
    include $(LOCAL_PATH)/Makefile.android
else
    include Makefile.qemu
endif
