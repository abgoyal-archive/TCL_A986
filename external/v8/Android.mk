BASE_PATH := $(call my-dir)
include $(CLEAR_VARS)

# Two ways to control which JS engine is used:
# 1. use JS_ENGINE environment variable, value can be either 'jsc' or 'v8'
#    This is the preferred way.
# 2. if JS_ENGINE is not set, or is not 'jsc' or 'v8', this makefile picks
#    up a default engine to build.
#    To help setup buildbot, a new environment variable, USE_ALT_JS_ENGINE,
#    can be set to true, so that two builds can be different but without
#    specifying which JS engine to use.

# Build libv8 and v8shell
# TODO: remove the check when the v8 build is fixed for x86
ifeq ($(TARGET_ARCH),arm)
    ENABLE_V8_SNAPSHOT = true
    include $(BASE_PATH)/Android.mksnapshot.mk
    include $(BASE_PATH)/Android.libv8.mk
    include $(BASE_PATH)/Android.v8shell.mk
endif
