
#ifndef __UINPUT_H_
#define __UINPUT_H_

#include <linux/input.h>

#define UINPUT_VERSION		3

#ifdef __KERNEL__
#define UINPUT_MINOR		223
#define UINPUT_NAME		"uinput"
#define UINPUT_BUFFER_SIZE	16
#define UINPUT_NUM_REQUESTS	16

enum uinput_state { UIST_NEW_DEVICE, UIST_SETUP_COMPLETE, UIST_CREATED };

struct uinput_request {
	int			id;
	int			code;	/* UI_FF_UPLOAD, UI_FF_ERASE */

	int			retval;
	struct completion	done;

	union {
		int		effect_id;
		struct {
			struct ff_effect *effect;
			struct ff_effect *old;
		} upload;
	} u;
};

struct uinput_device {
	struct input_dev	*dev;
	struct mutex		mutex;
	enum uinput_state	state;
	wait_queue_head_t	waitq;
	unsigned char		ready;
	unsigned char		head;
	unsigned char		tail;
	struct input_event	buff[UINPUT_BUFFER_SIZE];
	int			ff_effects_max;

	struct uinput_request	*requests[UINPUT_NUM_REQUESTS];
	wait_queue_head_t	requests_waitq;
	spinlock_t		requests_lock;
};
#endif	/* __KERNEL__ */

struct uinput_ff_upload {
	int			request_id;
	int			retval;
	struct ff_effect	effect;
	struct ff_effect	old;
};

struct uinput_ff_erase {
	int			request_id;
	int			retval;
	int			effect_id;
};

/* ioctl */
#define UINPUT_IOCTL_BASE	'U'
#define UI_DEV_CREATE		_IO(UINPUT_IOCTL_BASE, 1)
#define UI_DEV_DESTROY		_IO(UINPUT_IOCTL_BASE, 2)

#define UI_SET_EVBIT		_IOW(UINPUT_IOCTL_BASE, 100, int)
#define UI_SET_KEYBIT		_IOW(UINPUT_IOCTL_BASE, 101, int)
#define UI_SET_RELBIT		_IOW(UINPUT_IOCTL_BASE, 102, int)
#define UI_SET_ABSBIT		_IOW(UINPUT_IOCTL_BASE, 103, int)
#define UI_SET_MSCBIT		_IOW(UINPUT_IOCTL_BASE, 104, int)
#define UI_SET_LEDBIT		_IOW(UINPUT_IOCTL_BASE, 105, int)
#define UI_SET_SNDBIT		_IOW(UINPUT_IOCTL_BASE, 106, int)
#define UI_SET_FFBIT		_IOW(UINPUT_IOCTL_BASE, 107, int)
#define UI_SET_PHYS		_IOW(UINPUT_IOCTL_BASE, 108, char*)
#define UI_SET_SWBIT		_IOW(UINPUT_IOCTL_BASE, 109, int)

#define UI_BEGIN_FF_UPLOAD	_IOWR(UINPUT_IOCTL_BASE, 200, struct uinput_ff_upload)
#define UI_END_FF_UPLOAD	_IOW(UINPUT_IOCTL_BASE, 201, struct uinput_ff_upload)
#define UI_BEGIN_FF_ERASE	_IOWR(UINPUT_IOCTL_BASE, 202, struct uinput_ff_erase)
#define UI_END_FF_ERASE		_IOW(UINPUT_IOCTL_BASE, 203, struct uinput_ff_erase)


#define EV_UINPUT		0x0101
#define UI_FF_UPLOAD		1
#define UI_FF_ERASE		2

#define UINPUT_MAX_NAME_SIZE	80
struct uinput_user_dev {
	char name[UINPUT_MAX_NAME_SIZE];
	struct input_id id;
	int ff_effects_max;
	int absmax[ABS_CNT];
	int absmin[ABS_CNT];
	int absfuzz[ABS_CNT];
	int absflat[ABS_CNT];
};
#endif	/* __UINPUT_H_ */

