

#ifndef EC100_H
#define EC100_H

#include <linux/dvb/frontend.h>

struct ec100_config {
	/* demodulator's I2C address */
	u8 demod_address;
};


#if defined(CONFIG_DVB_EC100) || \
	(defined(CONFIG_DVB_EC100_MODULE) && defined(MODULE))
extern struct dvb_frontend *ec100_attach(const struct ec100_config *config,
	struct i2c_adapter *i2c);
#else
static inline struct dvb_frontend *ec100_attach(
	const struct ec100_config *config, struct i2c_adapter *i2c)
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}
#endif

#endif /* EC100_H */
