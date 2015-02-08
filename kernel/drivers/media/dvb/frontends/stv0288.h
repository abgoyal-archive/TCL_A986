

#ifndef STV0288_H
#define STV0288_H

#include <linux/dvb/frontend.h>
#include "dvb_frontend.h"

struct stv0288_config {
	/* the demodulator's i2c address */
	u8 demod_address;

	u8* inittab;

	/* minimum delay before retuning */
	int min_delay_ms;

	int (*set_ts_params)(struct dvb_frontend *fe, int is_punctured);
};

#if defined(CONFIG_DVB_STV0288) || (defined(CONFIG_DVB_STV0288_MODULE) && \
							defined(MODULE))
extern struct dvb_frontend *stv0288_attach(const struct stv0288_config *config,
					   struct i2c_adapter *i2c);
#else
static inline struct dvb_frontend *stv0288_attach(const struct stv0288_config *config,
					   struct i2c_adapter *i2c)
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}
#endif /* CONFIG_DVB_STV0288 */

static inline int stv0288_writereg(struct dvb_frontend *fe, u8 reg, u8 val)
{
	int r = 0;
	u8 buf[] = { reg, val };
	if (fe->ops.write)
		r = fe->ops.write(fe, buf, 2);
	return r;
}

#endif /* STV0288_H */
