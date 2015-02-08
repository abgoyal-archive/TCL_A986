

#ifndef DS3000_H
#define DS3000_H

#include <linux/dvb/frontend.h>

struct ds3000_config {
	/* the demodulator's i2c address */
	u8 demod_address;
};

#if defined(CONFIG_DVB_DS3000) || \
			(defined(CONFIG_DVB_DS3000_MODULE) && defined(MODULE))
extern struct dvb_frontend *ds3000_attach(const struct ds3000_config *config,
					struct i2c_adapter *i2c);
#else
static inline
struct dvb_frontend *ds3000_attach(const struct ds3000_config *config,
					struct i2c_adapter *i2c)
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}
#endif /* CONFIG_DVB_DS3000 */
#endif /* DS3000_H */
