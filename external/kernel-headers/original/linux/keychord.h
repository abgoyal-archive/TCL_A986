
#ifndef __LINUX_KEYCHORD_H_
#define __LINUX_KEYCHORD_H_

#include <linux/input.h>

#define KEYCHORD_VERSION		1

struct input_keychord {
	/* should be KEYCHORD_VERSION */
	__u16 version;
	/*
	 * client specified ID, returned from read()
	 * when this keychord is pressed.
	 */
	__u16 id;

	/* number of keycodes in this keychord */
	__u16 count;

	/* variable length array of keycodes */
	__u16 keycodes[];
};

#endif	/* __LINUX_KEYCHORD_H_ */
