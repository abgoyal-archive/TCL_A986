

#ifndef S921_MODULE_H
#define S921_MODULE_H

#include <linux/dvb/frontend.h>
#include "s921_core.h"

int s921_isdb_init(struct s921_isdb_t *dev);
int s921_isdb_cmd(struct s921_isdb_t *dev, u32 cmd, void *data);

struct s921_config
{
	/* demodulator's I2C address */
	u8 i2c_address;
};

#if defined(CONFIG_DVB_S921) || (defined(CONFIG_DVB_S921_MODULE) && defined(MODULE))
extern struct dvb_frontend* s921_attach(const struct s921_config *config,
					   struct i2c_adapter *i2c);
#else
static inline struct dvb_frontend* s921_attach(const struct s921_config *config,
					   struct i2c_adapter *i2c)
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}
#endif /* CONFIG_DVB_S921 */

#endif /* S921_H */
