

#ifndef _LNBH24_H
#define _LNBH24_H

/* system register bits */
#define LNBH24_OLF	0x01
#define LNBH24_OTF	0x02
#define LNBH24_EN	0x04
#define LNBH24_VSEL	0x08
#define LNBH24_LLC	0x10
#define LNBH24_TEN	0x20
#define LNBH24_TTX	0x40
#define LNBH24_PCL	0x80

#include <linux/dvb/frontend.h>

#if defined(CONFIG_DVB_LNBP21) || (defined(CONFIG_DVB_LNBP21_MODULE) \
							&& defined(MODULE))
extern struct dvb_frontend *lnbh24_attach(struct dvb_frontend *fe,
				struct i2c_adapter *i2c, u8 override_set,
				u8 override_clear, u8 i2c_addr);
#else
static inline struct dvb_frontend *lnbh24_attach(struct dvb_frontend *fe,
				struct i2c_adapter *i2c, u8 override_set,
				u8 override_clear, u8 i2c_addr)
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}
#endif

#endif
