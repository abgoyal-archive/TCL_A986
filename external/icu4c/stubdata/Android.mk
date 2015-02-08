LOCAL_PATH:= $(call my-dir)

# Build configuration:
#
# "Large" includes all the supported locales.
# Japanese includes US and Japan.
# US-Euro is needed for IT or PL builds
# Default is suitable for CS, DE, EN, ES, FR, NL
# US has only EN and ES

config := $(word 1, \
            $(if $(findstring ar,$(PRODUCT_LOCALES)),large) \
            $(if $(findstring da,$(PRODUCT_LOCALES)),large) \
            $(if $(findstring el,$(PRODUCT_LOCALES)),large) \
            $(if $(findstring fi,$(PRODUCT_LOCALES)),large) \
            $(if $(findstring he,$(PRODUCT_LOCALES)),large) \
            $(if $(findstring hr,$(PRODUCT_LOCALES)),large) \
            $(if $(findstring hu,$(PRODUCT_LOCALES)),large) \
            $(if $(findstring id,$(PRODUCT_LOCALES)),large) \
            $(if $(findstring ko,$(PRODUCT_LOCALES)),large) \
            $(if $(findstring nb,$(PRODUCT_LOCALES)),large) \
            $(if $(findstring pt,$(PRODUCT_LOCALES)),large) \
            $(if $(findstring ro,$(PRODUCT_LOCALES)),large) \
            $(if $(findstring ru,$(PRODUCT_LOCALES)),large) \
            $(if $(findstring sk,$(PRODUCT_LOCALES)),large) \
            $(if $(findstring sr,$(PRODUCT_LOCALES)),large) \
            $(if $(findstring sv,$(PRODUCT_LOCALES)),large) \
            $(if $(findstring th,$(PRODUCT_LOCALES)),large) \
            $(if $(findstring tr,$(PRODUCT_LOCALES)),large) \
            $(if $(findstring uk,$(PRODUCT_LOCALES)),large) \
            $(if $(findstring zh,$(PRODUCT_LOCALES)),large) \
            $(if $(findstring ja,$(PRODUCT_LOCALES)),us-japan) \
            $(if $(findstring it,$(PRODUCT_LOCALES)),us-euro) \
            $(if $(findstring pl,$(PRODUCT_LOCALES)),us-euro) \
            $(if $(findstring cs,$(PRODUCT_LOCALES)),default) \
            $(if $(findstring de,$(PRODUCT_LOCALES)),default) \
            $(if $(findstring fr,$(PRODUCT_LOCALES)),default) \
            $(if $(findstring nl,$(PRODUCT_LOCALES)),default) \
            us)

include $(LOCAL_PATH)/root.mk

PRODUCT_COPY_FILES += $(LOCAL_PATH)/$(root)-$(config).dat:/system/usr/icu/$(root).dat

ifeq ($(WITH_HOST_DALVIK),true)
    $(eval $(call copy-one-file,$(LOCAL_PATH)/$(root)-$(config).dat,$(HOST_OUT)/usr/icu/$(root).dat))
endif
