

#ifndef _LINUX_EARLYSUSPEND_H
#define _LINUX_EARLYSUSPEND_H

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/list.h>
#endif

enum {
	EARLY_SUSPEND_LEVEL_BLANK_SCREEN = 50,
	EARLY_SUSPEND_LEVEL_STOP_DRAWING = 100,
	EARLY_SUSPEND_LEVEL_DISABLE_FB = 150,
};
struct early_suspend {
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct list_head link;
	int level;
	void (*suspend)(struct early_suspend *h);
	void (*resume)(struct early_suspend *h);
#endif
};

#ifdef CONFIG_HAS_EARLYSUSPEND
void register_early_suspend(struct early_suspend *handler);
void unregister_early_suspend(struct early_suspend *handler);
#else
#define register_early_suspend(handler) do { } while (0)
#define unregister_early_suspend(handler) do { } while (0)
#endif

#endif

