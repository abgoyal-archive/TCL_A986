
#ifndef BCM3510_H
#define BCM3510_H

#include <linux/dvb/frontend.h>
#include <linux/firmware.h>

struct bcm3510_config
{
	/* the demodulator's i2c address */
	u8 demod_address;

	/* request firmware for device */
	int (*request_firmware)(struct dvb_frontend* fe, const struct firmware **fw, char* name);
};

#if defined(CONFIG_DVB_BCM3510) || (defined(CONFIG_DVB_BCM3510_MODULE) && defined(MODULE))
extern struct dvb_frontend* bcm3510_attach(const struct bcm3510_config* config,
					   struct i2c_adapter* i2c);
#else
static inline struct dvb_frontend* bcm3510_attach(const struct bcm3510_config* config,
						  struct i2c_adapter* i2c)
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}
#endif // CONFIG_DVB_BCM3510

#endif
