

#ifndef __NOUVEAU_I2C_H__
#define __NOUVEAU_I2C_H__

#include <linux/i2c.h>
#include <linux/i2c-id.h>
#include <linux/i2c-algo-bit.h>
#include "drm_dp_helper.h"

struct dcb_i2c_entry;

struct nouveau_i2c_chan {
	struct i2c_adapter adapter;
	struct drm_device *dev;
	union {
		struct i2c_algo_bit_data bit;
		struct i2c_algo_dp_aux_data dp;
	} algo;
	unsigned rd;
	unsigned wr;
	unsigned data;
};

int nouveau_i2c_init(struct drm_device *, struct dcb_i2c_entry *, int index);
void nouveau_i2c_fini(struct drm_device *, struct dcb_i2c_entry *);
struct nouveau_i2c_chan *nouveau_i2c_find(struct drm_device *, int index);

int nouveau_dp_i2c_aux_ch(struct i2c_adapter *, int mode, uint8_t write_byte,
			  uint8_t *read_byte);

#endif /* __NOUVEAU_I2C_H__ */
