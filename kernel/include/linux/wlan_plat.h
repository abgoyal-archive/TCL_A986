
#ifndef _LINUX_WLAN_PLAT_H_
#define _LINUX_WLAN_PLAT_H_

struct wifi_platform_data {
	int (*set_power)(int val);
	int (*set_reset)(int val);
	int (*set_carddetect)(int val);
	void *(*mem_prealloc)(int section, unsigned long size);
	int (*get_mac_addr)(unsigned char *buf);
};

#endif
