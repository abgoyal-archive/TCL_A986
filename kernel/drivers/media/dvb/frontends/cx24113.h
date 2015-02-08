

#ifndef CX24113_H
#define CX24113_H

struct dvb_frontend;

struct cx24113_config {
	u8 i2c_addr; /* 0x14 or 0x54 */

	u32 xtal_khz;
};

#if defined(CONFIG_DVB_TUNER_CX24113) || \
	(defined(CONFIG_DVB_TUNER_CX24113_MODULE) && defined(MODULE))
extern struct dvb_frontend *cx24113_attach(struct dvb_frontend *,
	const struct cx24113_config *config, struct i2c_adapter *i2c);

extern void cx24113_agc_callback(struct dvb_frontend *fe);
#else
static inline struct dvb_frontend *cx24113_attach(struct dvb_frontend *fe,
	const struct cx24113_config *config, struct i2c_adapter *i2c)
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}

static inline void cx24113_agc_callback(struct dvb_frontend *fe)
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
}
#endif

#endif /* CX24113_H */
