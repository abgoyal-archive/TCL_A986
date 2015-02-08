

#ifndef __LINUX_LM8323_H
#define __LINUX_LM8323_H

#include <linux/types.h>

#define LM8323_KEYMAP_SIZE	(0x7f + 1)

#define LM8323_NUM_PWMS		3

struct lm8323_platform_data {
	int debounce_time; /* Time to watch for key bouncing, in ms. */
	int active_time; /* Idle time until sleep, in ms. */

	int size_x;
	int size_y;
	bool repeat;
	const unsigned short *keymap;

	const char *pwm_names[LM8323_NUM_PWMS];

	const char *name; /* Device name. */
};

#endif /* __LINUX_LM8323_H */
