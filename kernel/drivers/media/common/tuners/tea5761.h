

#ifndef __TEA5761_H__
#define __TEA5761_H__

#include <linux/i2c.h>
#include "dvb_frontend.h"

#if defined(CONFIG_MEDIA_TUNER_TEA5761) || (defined(CONFIG_MEDIA_TUNER_TEA5761_MODULE) && defined(MODULE))
extern int tea5761_autodetection(struct i2c_adapter* i2c_adap, u8 i2c_addr);

extern struct dvb_frontend *tea5761_attach(struct dvb_frontend *fe,
					   struct i2c_adapter* i2c_adap,
					   u8 i2c_addr);
#else
static inline int tea5761_autodetection(struct i2c_adapter* i2c_adap,
					u8 i2c_addr)
{
	printk(KERN_INFO "%s: not probed - driver disabled by Kconfig\n",
	       __func__);
	return -EINVAL;
}

static inline struct dvb_frontend *tea5761_attach(struct dvb_frontend *fe,
						   struct i2c_adapter* i2c_adap,
						   u8 i2c_addr)
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}
#endif

#endif /* __TEA5761_H__ */
