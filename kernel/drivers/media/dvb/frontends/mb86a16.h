

#ifndef __MB86A16_H
#define __MB86A16_H

#include <linux/dvb/frontend.h>
#include "dvb_frontend.h"


struct mb86a16_config {
	u8 demod_address;

	int (*set_voltage)(struct dvb_frontend *fe, fe_sec_voltage_t voltage);
};



#if defined(CONFIG_DVB_MB86A16) || (defined(CONFIG_DVB_MB86A16_MODULE) && defined(MODULE))

extern struct dvb_frontend *mb86a16_attach(const struct mb86a16_config *config,
					   struct i2c_adapter *i2c_adap);

#else

static inline struct dvb_frontend *mb86a16_attach(const struct mb86a16_config *config,
					   struct i2c_adapter *i2c_adap)
{
	printk(KERN_WARNING "%s: Driver disabled by Kconfig\n", __func__);
	return NULL;
}

#endif /* CONFIG_DVB_MB86A16 */

#endif /* __MB86A16_H */
