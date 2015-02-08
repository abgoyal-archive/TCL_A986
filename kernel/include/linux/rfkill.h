
#ifndef __RFKILL_H
#define __RFKILL_H


#include <linux/types.h>

/* define userspace visible states */
#define RFKILL_STATE_SOFT_BLOCKED	0
#define RFKILL_STATE_UNBLOCKED		1
#define RFKILL_STATE_HARD_BLOCKED	2

enum rfkill_type {
	RFKILL_TYPE_ALL = 0,
	RFKILL_TYPE_WLAN,
	RFKILL_TYPE_BLUETOOTH,
	RFKILL_TYPE_UWB,
	RFKILL_TYPE_WIMAX,
	RFKILL_TYPE_WWAN,
	RFKILL_TYPE_GPS,
	RFKILL_TYPE_FM,
	NUM_RFKILL_TYPES,
};

enum rfkill_operation {
	RFKILL_OP_ADD = 0,
	RFKILL_OP_DEL,
	RFKILL_OP_CHANGE,
	RFKILL_OP_CHANGE_ALL,
};

struct rfkill_event {
	__u32 idx;
	__u8  type;
	__u8  op;
	__u8  soft, hard;
} __packed;

#define RFKILL_EVENT_SIZE_V1	8

/* ioctl for turning off rfkill-input (if present) */
#define RFKILL_IOC_MAGIC	'R'
#define RFKILL_IOC_NOINPUT	1
#define RFKILL_IOCTL_NOINPUT	_IO(RFKILL_IOC_MAGIC, RFKILL_IOC_NOINPUT)

/* and that's all userspace gets */
#ifdef __KERNEL__
/* don't allow anyone to use these in the kernel */
enum rfkill_user_states {
	RFKILL_USER_STATE_SOFT_BLOCKED	= RFKILL_STATE_SOFT_BLOCKED,
	RFKILL_USER_STATE_UNBLOCKED	= RFKILL_STATE_UNBLOCKED,
	RFKILL_USER_STATE_HARD_BLOCKED	= RFKILL_STATE_HARD_BLOCKED,
};
#undef RFKILL_STATE_SOFT_BLOCKED
#undef RFKILL_STATE_UNBLOCKED
#undef RFKILL_STATE_HARD_BLOCKED

#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/leds.h>
#include <linux/err.h>

/* this is opaque */
struct rfkill;

struct rfkill_ops {
	void	(*poll)(struct rfkill *rfkill, void *data);
	void	(*query)(struct rfkill *rfkill, void *data);
	int	(*set_block)(void *data, bool blocked);
};

#if defined(CONFIG_RFKILL) || defined(CONFIG_RFKILL_MODULE)
struct rfkill * __must_check rfkill_alloc(const char *name,
					  struct device *parent,
					  const enum rfkill_type type,
					  const struct rfkill_ops *ops,
					  void *ops_data);

int __must_check rfkill_register(struct rfkill *rfkill);

void rfkill_pause_polling(struct rfkill *rfkill);

void rfkill_resume_polling(struct rfkill *rfkill);


void rfkill_unregister(struct rfkill *rfkill);

void rfkill_destroy(struct rfkill *rfkill);

bool rfkill_set_hw_state(struct rfkill *rfkill, bool blocked);

bool rfkill_set_sw_state(struct rfkill *rfkill, bool blocked);

void rfkill_init_sw_state(struct rfkill *rfkill, bool blocked);

void rfkill_set_states(struct rfkill *rfkill, bool sw, bool hw);

bool rfkill_blocked(struct rfkill *rfkill);
#else /* !RFKILL */
static inline struct rfkill * __must_check
rfkill_alloc(const char *name,
	     struct device *parent,
	     const enum rfkill_type type,
	     const struct rfkill_ops *ops,
	     void *ops_data)
{
	return ERR_PTR(-ENODEV);
}

static inline int __must_check rfkill_register(struct rfkill *rfkill)
{
	if (rfkill == ERR_PTR(-ENODEV))
		return 0;
	return -EINVAL;
}

static inline void rfkill_pause_polling(struct rfkill *rfkill)
{
}

static inline void rfkill_resume_polling(struct rfkill *rfkill)
{
}

static inline void rfkill_unregister(struct rfkill *rfkill)
{
}

static inline void rfkill_destroy(struct rfkill *rfkill)
{
}

static inline bool rfkill_set_hw_state(struct rfkill *rfkill, bool blocked)
{
	return blocked;
}

static inline bool rfkill_set_sw_state(struct rfkill *rfkill, bool blocked)
{
	return blocked;
}

static inline void rfkill_init_sw_state(struct rfkill *rfkill, bool blocked)
{
}

static inline void rfkill_set_states(struct rfkill *rfkill, bool sw, bool hw)
{
}

static inline bool rfkill_blocked(struct rfkill *rfkill)
{
	return false;
}
#endif /* RFKILL || RFKILL_MODULE */


#ifdef CONFIG_RFKILL_LEDS
const char *rfkill_get_led_trigger_name(struct rfkill *rfkill);

void rfkill_set_led_trigger_name(struct rfkill *rfkill, const char *name);
#else
static inline const char *rfkill_get_led_trigger_name(struct rfkill *rfkill)
{
	return NULL;
}

static inline void
rfkill_set_led_trigger_name(struct rfkill *rfkill, const char *name)
{
}
#endif

#endif /* __KERNEL__ */

#endif /* RFKILL_H */
