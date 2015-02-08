

#ifndef __DVB_TUA6100_H__
#define __DVB_TUA6100_H__

#include <linux/i2c.h>
#include "dvb_frontend.h"

#if defined(CONFIG_DVB_TUA6100) || (defined(CONFIG_DVB_TUA6100_MODULE) && defined(MODULE))
extern struct dvb_frontend *tua6100_attach(struct dvb_frontend *fe, int addr, struct i2c_adapter *i2c);
#else
static inline struct dvb_frontend* tua6100_attach(struct dvb_frontend *fe, int addr, struct i2c_adapter *i2c)
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}
#endif // CONFIG_DVB_TUA6100

#endif
