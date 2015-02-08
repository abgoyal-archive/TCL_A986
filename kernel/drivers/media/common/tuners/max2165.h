

#ifndef __MAX2165_H__
#define __MAX2165_H__

struct dvb_frontend;
struct i2c_adapter;

struct max2165_config {
	u8 i2c_address;
	u8 osc_clk; /* in MHz, selectable values: 4,16,18,20,22,24,26,28 */
};

#if defined(CONFIG_MEDIA_TUNER_MAX2165) || \
    (defined(CONFIG_MEDIA_TUNER_MAX2165_MODULE) && defined(MODULE))
extern struct dvb_frontend *max2165_attach(struct dvb_frontend *fe,
	struct i2c_adapter *i2c,
	struct max2165_config *cfg);
#else
static inline struct dvb_frontend *max2165_attach(struct dvb_frontend *fe,
	struct i2c_adapter *i2c,
	struct max2165_config *cfg)
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}
#endif

#endif
