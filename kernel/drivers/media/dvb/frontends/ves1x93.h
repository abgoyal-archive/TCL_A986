

#ifndef VES1X93_H
#define VES1X93_H

#include <linux/dvb/frontend.h>

struct ves1x93_config
{
	/* the demodulator's i2c address */
	u8 demod_address;

	/* value of XIN to use */
	u32 xin;

	/* should PWM be inverted? */
	u8 invert_pwm:1;
};

#if defined(CONFIG_DVB_VES1X93) || (defined(CONFIG_DVB_VES1X93_MODULE) && defined(MODULE))
extern struct dvb_frontend* ves1x93_attach(const struct ves1x93_config* config,
					   struct i2c_adapter* i2c);
#else
static inline struct dvb_frontend* ves1x93_attach(const struct ves1x93_config* config,
					   struct i2c_adapter* i2c)
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}
#endif // CONFIG_DVB_VES1X93

#endif // VES1X93_H
