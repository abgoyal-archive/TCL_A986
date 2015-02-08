

#ifndef CX22702_H
#define CX22702_H

#include <linux/dvb/frontend.h>

struct cx22702_config {
	/* the demodulator's i2c address */
	u8 demod_address;

	/* serial/parallel output */
#define CX22702_PARALLEL_OUTPUT 0
#define CX22702_SERIAL_OUTPUT   1
	u8 output_mode;
};

#if defined(CONFIG_DVB_CX22702) || (defined(CONFIG_DVB_CX22702_MODULE) \
	&& defined(MODULE))
extern struct dvb_frontend *cx22702_attach(
	const struct cx22702_config *config,
	struct i2c_adapter *i2c);
#else
static inline struct dvb_frontend *cx22702_attach(
	const struct cx22702_config *config,
	struct i2c_adapter *i2c)
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}
#endif

#endif
