

#ifndef LGS8GL5_H
#define LGS8GL5_H

#include <linux/dvb/frontend.h>

struct lgs8gl5_config {
	/* the demodulator's i2c address */
	u8 demod_address;
};

#if defined(CONFIG_DVB_LGS8GL5) || \
	(defined(CONFIG_DVB_LGS8GL5_MODULE) && defined(MODULE))
extern struct dvb_frontend *lgs8gl5_attach(
	const struct lgs8gl5_config *config, struct i2c_adapter *i2c);
#else
static inline struct dvb_frontend *lgs8gl5_attach(
	const struct lgs8gl5_config *config, struct i2c_adapter *i2c) {
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}
#endif /* CONFIG_DVB_LGS8GL5 */

#endif /* LGS8GL5_H */
