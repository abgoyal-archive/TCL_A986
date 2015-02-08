

#ifndef __LINUX_SND_WM_9081_H
#define __LINUX_SND_WM_9081_H

struct wm9081_retune_mobile_setting {
	const char *name;
	unsigned int rate;
	u16 config[20];
};

struct wm9081_retune_mobile_config {
	struct wm9081_retune_mobile_setting *configs;
	int num_configs;
};

#endif
