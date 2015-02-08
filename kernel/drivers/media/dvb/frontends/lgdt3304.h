

#ifndef LGDT3304_H
#define LGDT3304_H

#include <linux/dvb/frontend.h>

struct lgdt3304_config
{
	/* demodulator's I2C address */
	u8 i2c_address;
};

#if defined(CONFIG_DVB_LGDT3304) || (defined(CONFIG_DVB_LGDT3304_MODULE) && defined(MODULE))
extern struct dvb_frontend* lgdt3304_attach(const struct lgdt3304_config *config,
					   struct i2c_adapter *i2c);
#else
static inline struct dvb_frontend* lgdt3304_attach(const struct lgdt3304_config *config,
					   struct i2c_adapter *i2c)
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}
#endif /* CONFIG_DVB_LGDT */

#endif /* LGDT3304_H */
