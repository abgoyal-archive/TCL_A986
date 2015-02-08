

#ifndef MT312_H
#define MT312_H

#include <linux/dvb/frontend.h>

struct mt312_config {
	/* the demodulator's i2c address */
	u8 demod_address;

	/* inverted voltage setting */
	unsigned int voltage_inverted:1;
};

#if defined(CONFIG_DVB_MT312) || (defined(CONFIG_DVB_MT312_MODULE) && defined(MODULE))
struct dvb_frontend *mt312_attach(const struct mt312_config *config,
					struct i2c_adapter *i2c);
#else
static inline struct dvb_frontend *mt312_attach(
	const struct mt312_config *config, struct i2c_adapter *i2c)
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}
#endif /* CONFIG_DVB_MT312 */

#endif /* MT312_H */
