ifneq ($(TARGET_SIMULATOR),true)

LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

LOCAL_SRC_FILES := \
	src/libipsec/pfkey.c \
	src/libipsec/ipsec_strerror.c \
	src/racoon/isakmp.c \
	src/racoon/isakmp_agg.c \
	src/racoon/isakmp_base.c \
	src/racoon/isakmp_frag.c \
	src/racoon/isakmp_ident.c \
	src/racoon/isakmp_inf.c \
	src/racoon/isakmp_newg.c \
	src/racoon/isakmp_quick.c \
	src/racoon/handler.c \
	src/racoon/pfkey.c \
	src/racoon/ipsec_doi.c \
	src/racoon/oakley.c \
	src/racoon/vendorid.c \
	src/racoon/policy.c \
	src/racoon/crypto_openssl.c \
	src/racoon/algorithm.c \
	src/racoon/proposal.c \
	src/racoon/strnames.c \
	src/racoon/schedule.c \
	src/racoon/str2val.c \
	src/racoon/genlist.c \
	src/racoon/vmbuf.c \
	src/racoon/sockmisc.c \
	src/racoon/nattraversal.c \
	main.c \
	setup.c

LOCAL_C_INCLUDES += \
	$(LOCAL_PATH) \
	$(LOCAL_PATH)/src/include-glibc \
	$(LOCAL_PATH)/src/libipsec \
	$(LOCAL_PATH)/src/racoon \
	$(LOCAL_PATH)/src/racoon/missing \
	external/openssl/include \
	frameworks/base/cmds/keystore

LOCAL_SHARED_LIBRARIES := libcutils libcrypto

LOCAL_CFLAGS := -DANDROID_CHANGES -DHAVE_CONFIG_H

LOCAL_MODULE := racoon

include $(BUILD_EXECUTABLE)

endif
