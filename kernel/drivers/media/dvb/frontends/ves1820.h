

#ifndef VES1820_H
#define VES1820_H

#include <linux/dvb/frontend.h>

#define VES1820_SELAGC_PWM 0
#define VES1820_SELAGC_SIGNAMPERR 1

struct ves1820_config
{
	/* the demodulator's i2c address */
	u8 demod_address;

	/* value of XIN to use */
	u32 xin;

	/* does inversion need inverted? */
	u8 invert:1;

	/* SELAGC control */
	u8 selagc:1;
};

#if defined(CONFIG_DVB_VES1820) || (defined(CONFIG_DVB_VES1820_MODULE) && defined(MODULE))
extern struct dvb_frontend* ves1820_attach(const struct ves1820_config* config,
					   struct i2c_adapter* i2c, u8 pwm);
#else
static inline struct dvb_frontend* ves1820_attach(const struct ves1820_config* config,
					   struct i2c_adapter* i2c, u8 pwm)
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}
#endif // CONFIG_DVB_VES1820

#endif // VES1820_H
