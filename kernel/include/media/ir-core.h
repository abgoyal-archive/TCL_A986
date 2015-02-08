

#ifndef _IR_CORE
#define _IR_CORE

#include <linux/spinlock.h>
#include <linux/kfifo.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <media/rc-map.h>

extern int ir_core_debug;
#define IR_dprintk(level, fmt, arg...)	if (ir_core_debug >= level) \
	printk(KERN_DEBUG "%s: " fmt , __func__, ## arg)

enum rc_driver_type {
	RC_DRIVER_SCANCODE = 0,	/* Driver or hardware generates a scancode */
	RC_DRIVER_IR_RAW,	/* Needs a Infra-Red pulse/space decoder */
};

struct ir_dev_props {
	enum rc_driver_type	driver_type;
	unsigned long		allowed_protos;
	u32			scanmask;
	void 			*priv;
	int			(*change_protocol)(void *priv, u64 ir_type);
	int			(*open)(void *priv);
	void			(*close)(void *priv);
};

struct ir_input_dev {
	struct device			dev;		/* device */
	char				*driver_name;	/* Name of the driver module */
	struct ir_scancode_table	rc_tab;		/* scan/key table */
	unsigned long			devno;		/* device number */
	const struct ir_dev_props	*props;		/* Device properties */
	struct ir_raw_event_ctrl	*raw;		/* for raw pulse/space events */
	struct input_dev		*input_dev;	/* the input device associated with this device */

	/* key info - needed by IR keycode handlers */
	spinlock_t			keylock;	/* protects the below members */
	bool				keypressed;	/* current state */
	unsigned long			keyup_jiffies;	/* when should the current keypress be released? */
	struct timer_list		timer_keyup;	/* timer for releasing a keypress */
	u32				last_keycode;	/* keycode of last command */
	u32				last_scancode;	/* scancode of last command */
	u8				last_toggle;	/* toggle of last command */
};

enum raw_event_type {
	IR_SPACE        = (1 << 0),
	IR_PULSE        = (1 << 1),
	IR_START_EVENT  = (1 << 2),
	IR_STOP_EVENT   = (1 << 3),
};

#define to_ir_input_dev(_attr) container_of(_attr, struct ir_input_dev, attr)

/* From ir-keytable.c */
int __ir_input_register(struct input_dev *dev,
		      const struct ir_scancode_table *ir_codes,
		      const struct ir_dev_props *props,
		      const char *driver_name);

static inline int ir_input_register(struct input_dev *dev,
		      const char *map_name,
		      const struct ir_dev_props *props,
		      const char *driver_name) {
	struct ir_scancode_table *ir_codes;
	struct ir_input_dev *ir_dev;
	int rc;

	if (!map_name)
		return -EINVAL;

	ir_codes = get_rc_map(map_name);
	if (!ir_codes)
		return -EINVAL;

	rc = __ir_input_register(dev, ir_codes, props, driver_name);
	if (rc < 0)
		return -EINVAL;

	ir_dev = input_get_drvdata(dev);

	if (!rc && ir_dev->props && ir_dev->props->change_protocol)
		rc = ir_dev->props->change_protocol(ir_dev->props->priv,
						    ir_codes->ir_type);

	return rc;
}

void ir_input_unregister(struct input_dev *input_dev);

void ir_repeat(struct input_dev *dev);
void ir_keydown(struct input_dev *dev, int scancode, u8 toggle);
u32 ir_g_keycode_from_table(struct input_dev *input_dev, u32 scancode);

/* From ir-raw-event.c */

struct ir_raw_event {
	unsigned                        pulse:1;
	unsigned                        duration:31;
};

#define IR_MAX_DURATION                 0x7FFFFFFF      /* a bit more than 2 seconds */

void ir_raw_event_handle(struct input_dev *input_dev);
int ir_raw_event_store(struct input_dev *input_dev, struct ir_raw_event *ev);
int ir_raw_event_store_edge(struct input_dev *input_dev, enum raw_event_type type);
static inline void ir_raw_event_reset(struct input_dev *input_dev)
{
	struct ir_raw_event ev = { .pulse = false, .duration = 0 };
	ir_raw_event_store(input_dev, &ev);
	ir_raw_event_handle(input_dev);
}

#endif /* _IR_CORE */
