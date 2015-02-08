

#ifndef _LNBP21_H
#define _LNBP21_H

/* system register bits */
/* [RO] 0=OK; 1=over current limit flag */
#define LNBP21_OLF	0x01
/* [RO] 0=OK; 1=over temperature flag (150 C) */
#define LNBP21_OTF	0x02
#define LNBP21_EN	0x04
#define LNBP21_VSEL	0x08
#define LNBP21_LLC	0x10
#define LNBP21_TEN	0x20
#define LNBP21_ISEL	0x40
#define LNBP21_PCL	0x80

#include <linux/dvb/frontend.h>

#if defined(CONFIG_DVB_LNBP21) || (defined(CONFIG_DVB_LNBP21_MODULE) \
							&& defined(MODULE))
extern struct dvb_frontend *lnbp21_attach(struct dvb_frontend *fe,
				struct i2c_adapter *i2c, u8 override_set,
				u8 override_clear);
#else
static inline struct dvb_frontend *lnbp21_attach(struct dvb_frontend *fe,
				struct i2c_adapter *i2c, u8 override_set,
				u8 override_clear)
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}
#endif

#endif
