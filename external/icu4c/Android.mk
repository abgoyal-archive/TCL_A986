LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

subdirs := $(addprefix $(LOCAL_PATH)/,$(addsuffix /Android.mk, \
		common   \
		i18n     \
		stubdata \
	))

include $(subdirs)
