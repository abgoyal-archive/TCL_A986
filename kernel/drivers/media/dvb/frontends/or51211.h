

#ifndef OR51211_H
#define OR51211_H

#include <linux/dvb/frontend.h>
#include <linux/firmware.h>

struct or51211_config
{
	/* The demodulator's i2c address */
	u8 demod_address;

	/* Request firmware for device */
	int (*request_firmware)(struct dvb_frontend* fe, const struct firmware **fw, char* name);
	void (*setmode)(struct dvb_frontend * fe, int mode);
	void (*reset)(struct dvb_frontend * fe);
	void (*sleep)(struct dvb_frontend * fe);
};

#if defined(CONFIG_DVB_OR51211) || (defined(CONFIG_DVB_OR51211_MODULE) && defined(MODULE))
extern struct dvb_frontend* or51211_attach(const struct or51211_config* config,
					   struct i2c_adapter* i2c);
#else
static inline struct dvb_frontend* or51211_attach(const struct or51211_config* config,
					   struct i2c_adapter* i2c)
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}
#endif // CONFIG_DVB_OR51211

#endif // OR51211_H

