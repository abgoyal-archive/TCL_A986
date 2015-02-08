
#ifndef S5H1420_H
#define S5H1420_H

#include <linux/dvb/frontend.h>

struct s5h1420_config
{
	/* the demodulator's i2c address */
	u8 demod_address;

	/* does the inversion require inversion? */
	u8 invert:1;

	u8 repeated_start_workaround:1;
	u8 cdclk_polarity:1; /* 1 == falling edge, 0 == raising edge */

	u8 serial_mpeg:1;
};

#if defined(CONFIG_DVB_S5H1420) || (defined(CONFIG_DVB_S5H1420_MODULE) && defined(MODULE))
extern struct dvb_frontend *s5h1420_attach(const struct s5h1420_config *config,
	     struct i2c_adapter *i2c);
extern struct i2c_adapter *s5h1420_get_tuner_i2c_adapter(struct dvb_frontend *fe);
#else
static inline struct dvb_frontend *s5h1420_attach(const struct s5h1420_config *config,
					   struct i2c_adapter *i2c)
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}

static inline struct i2c_adapter *s5h1420_get_tuner_i2c_adapter(struct dvb_frontend *fe)
{
	return NULL;
}
#endif // CONFIG_DVB_S5H1420

#endif // S5H1420_H
