

#ifndef ZL10039_H
#define ZL10039_H

#if defined(CONFIG_DVB_ZL10039) || (defined(CONFIG_DVB_ZL10039_MODULE) \
	    && defined(MODULE))
struct dvb_frontend *zl10039_attach(struct dvb_frontend *fe,
					u8 i2c_addr,
					struct i2c_adapter *i2c);
#else
static inline struct dvb_frontend *zl10039_attach(struct dvb_frontend *fe,
					u8 i2c_addr,
					struct i2c_adapter *i2c)
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}
#endif /* CONFIG_DVB_ZL10039 */

#endif /* ZL10039_H */
