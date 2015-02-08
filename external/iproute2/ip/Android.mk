LOCAL_PATH := $(call my-dir)

etc_dir := $(TARGET_OUT)/etc/iproute2

include $(CLEAR_VARS)

LOCAL_C_INCLUDES := $(KERNEL_HEADERS) \
					$(LOCAL_PATH)/../include/ \
					$(LOCAL_PATH)/../include/linux/ \
					
LOCAL_SRC_FILES :=  ip.c ip6tunnel.c ipaddress.c ipaddrlabel.c iplink.c iplink_can.c \
                    iplink_vlan.c ipmaddr.c ipmonitor.c ipmroute.c ipneigh.c \
                    ipntable.c iproute.c ipprefix.c iprule.c iptunnel.c ipxfrm.c \
                    link_gre.c link_veth.c tunnel.c xfrm_monitor.c xfrm_policy.c xfrm_state.c \
                    rtm_map.c

LOCAL_MODULE := ip
LOCAL_MODULE_TAGS := 

LOCAL_SYSTEM_SHARED_LIBRARIES := \
	libc libm libdl

LOCAL_SHARED_LIBRARIES += libiprouteutil libnetlink libcutils

LOCAL_CFLAGS := -O2 -g -W -Wall 

include $(BUILD_EXECUTABLE)

include $(CLEAR_VARS)
LOCAL_MODULE := rt_tables
LOCAL_MODULE_TAGS := user
LOCAL_MODULE_CLASS := ETC
LOCAL_MODULE_PATH := $(etc_dir)
LOCAL_SRC_FILES := rt_tables
include $(BUILD_PREBUILT)
