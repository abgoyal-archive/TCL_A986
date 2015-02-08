ifeq ($(MTK_NEW_IPTABLES_SUPPORT), yes)
ifneq ($(TARGET_SIMULATOR),true)
  BUILD_IPTABLES := 1
endif
ifeq ($(BUILD_IPTABLES),1)

LOCAL_PATH:= $(call my-dir)

#
# Build libraries
#

# libiptc

include $(CLEAR_VARS)

LOCAL_C_INCLUDES:= \
	$(KERNEL_HEADERS) \
	$(LOCAL_PATH)/include/

LOCAL_CFLAGS:=-DNO_SHARED_LIBS

LOCAL_SRC_FILES:= \
	libiptc/libip4tc.c

LOCAL_MODULE_TAGS:=
LOCAL_MODULE:=libiptc

include $(BUILD_STATIC_LIBRARY)

# libext

include $(CLEAR_VARS)

LOCAL_MODULE_TAGS:=
LOCAL_MODULE:=libext

# LOCAL_MODULE_CLASS must be defined before calling $(local-intermediates-dir)
#
LOCAL_MODULE_CLASS := STATIC_LIBRARIES
intermediates := $(call local-intermediates-dir)

LOCAL_C_INCLUDES:= \
	$(LOCAL_PATH)/include/ \
	$(KERNEL_HEADERS) \
	$(intermediates)/extensions/

LOCAL_CFLAGS:=-DNO_SHARED_LIBS
LOCAL_CFLAGS+=-D_INIT=$*_init
LOCAL_CFLAGS+=-DIPTABLES_VERSION=\"1.4.10\"

PF_EXT_SLIB:=ah addrtype
PF_EXT_SLIB+=icmp #2mark
PF_EXT_SLIB+=realm 
PF_EXT_SLIB+=unclean DNAT LOG #DSCP ECN
PF_EXT_SLIB+=MASQUERADE MIRROR NETMAP REDIRECT REJECT #MARK
PF_EXT_SLIB+=SAME SNAT ULOG # TOS TCPMSS TTL

PF_EXT_SLIB2:=comment connmark conntrack esp 
PF_EXT_SLIB2+=hashlimit helper iprange length limit mac multiport #2mark
PF_EXT_SLIB2+=owner physdev pkttype policy standard state tcp time
PF_EXT_SLIB2+=udp CLASSIFY #DSCP ECN
PF_EXT_SLIB2+=NFQUEUE NOTRACK #MARK


EXT_FUNC+=$(foreach T,$(PF_EXT_SLIB),ipt_$(T))
EXT_FUNC+=$(foreach T,$(PF_EXT_SLIB2),xt_$(T))

LOCAL_SRC_FILES:= \
	$(foreach T,$(PF_EXT_SLIB),extensions/libipt_$(T).c) \
	$(foreach T,$(PF_EXT_SLIB2),extensions/libxt_$(T).c) \
	extensions/initext4.c \
	extensions/initext6.c \
	xtables.c \
	xshared.c
	
LOCAL_STATIC_LIBRARIES := \
	libc

include $(BUILD_STATIC_LIBRARY)

#
# Build iptables
#

include $(CLEAR_VARS)

LOCAL_C_INCLUDES:= \
	$(LOCAL_PATH)/include/ \
	$(KERNEL_HEADERS)

LOCAL_CFLAGS:=-DNO_SHARED_LIBS
LOCAL_CFLAGS+=-DIPTABLES_VERSION=\"1.4.10\" # -DIPT_LIB_DIR=\"$(IPT_LIBDIR)\"
#LOCAL_CFLAGS+=-DIPT_LIB_DIR=\"$(IPT_LIBDIR)\"

LOCAL_SRC_FILES:= \
	iptables.c \
	iptables-standalone.c 

LOCAL_MODULE_TAGS:=
LOCAL_MODULE:=iptables

LOCAL_STATIC_LIBRARIES := \
	libiptc \
	libext

include $(BUILD_EXECUTABLE)

endif
endif
