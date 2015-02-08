

#ifndef L64781_H
#define L64781_H

#include <linux/dvb/frontend.h>

struct l64781_config
{
	/* the demodulator's i2c address */
	u8 demod_address;
};

#if defined(CONFIG_DVB_L64781) || (defined(CONFIG_DVB_L64781_MODULE) && defined(MODULE))
extern struct dvb_frontend* l64781_attach(const struct l64781_config* config,
					  struct i2c_adapter* i2c);
#else
static inline struct dvb_frontend* l64781_attach(const struct l64781_config* config,
					  struct i2c_adapter* i2c)
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}
#endif // CONFIG_DVB_L64781

#endif // L64781_H
