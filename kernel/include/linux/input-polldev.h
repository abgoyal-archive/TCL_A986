
#ifndef _INPUT_POLLDEV_H
#define _INPUT_POLLDEV_H


#include <linux/input.h>
#include <linux/workqueue.h>

struct input_polled_dev {
	void *private;

	void (*open)(struct input_polled_dev *dev);
	void (*close)(struct input_polled_dev *dev);
	void (*poll)(struct input_polled_dev *dev);
	unsigned int poll_interval; /* msec */
	unsigned int poll_interval_max; /* msec */
	unsigned int poll_interval_min; /* msec */

	struct input_dev *input;

/* private: */
	struct delayed_work work;
};

struct input_polled_dev *input_allocate_polled_device(void);
void input_free_polled_device(struct input_polled_dev *dev);
int input_register_polled_device(struct input_polled_dev *dev);
void input_unregister_polled_device(struct input_polled_dev *dev);

#endif
