

#ifndef TDA8083_H
#define TDA8083_H

#include <linux/dvb/frontend.h>

struct tda8083_config
{
	/* the demodulator's i2c address */
	u8 demod_address;
};

#if defined(CONFIG_DVB_TDA8083) || (defined(CONFIG_DVB_TDA8083_MODULE) && defined(MODULE))
extern struct dvb_frontend* tda8083_attach(const struct tda8083_config* config,
					   struct i2c_adapter* i2c);
#else
static inline struct dvb_frontend* tda8083_attach(const struct tda8083_config* config,
					   struct i2c_adapter* i2c)
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}
#endif // CONFIG_DVB_TDA8083

#endif // TDA8083_H
