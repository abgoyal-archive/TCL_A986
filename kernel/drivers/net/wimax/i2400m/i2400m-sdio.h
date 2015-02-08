

#ifndef __I2400M_SDIO_H__
#define __I2400M_SDIO_H__

#include "i2400m.h"

/* Host-Device interface for SDIO */
enum {
	I2400M_SDIO_BOOT_RETRIES = 3,
	I2400MS_BLK_SIZE = 256,
	I2400MS_PL_SIZE_MAX = 0x3E00,

	I2400MS_DATA_ADDR = 0x0,
	I2400MS_INTR_STATUS_ADDR = 0x13,
	I2400MS_INTR_CLEAR_ADDR = 0x13,
	I2400MS_INTR_ENABLE_ADDR = 0x14,
	I2400MS_INTR_GET_SIZE_ADDR = 0x2C,
	/* The number of ticks to wait for the device to signal that
	 * it is ready */
	I2400MS_INIT_SLEEP_INTERVAL = 100,
	/* How long to wait for the device to settle after reset */
	I2400MS_SETTLE_TIME = 40,
	/* The number of msec to wait for IOR after sending IOE */
	IWMC3200_IOR_TIMEOUT = 10,
};


struct i2400ms {
	struct i2400m i2400m;		/* FIRST! See doc */
	struct sdio_func *func;

	struct work_struct tx_worker;
	struct workqueue_struct *tx_workqueue;
	char tx_wq_name[32];

	struct dentry *debugfs_dentry;

	wait_queue_head_t bm_wfa_wq;
	int bm_wait_result;
	size_t bm_ack_size;

	/* Device is any of the iwmc3200 SKUs */
	unsigned iwmc3200:1;
};


static inline
void i2400ms_init(struct i2400ms *i2400ms)
{
	i2400m_init(&i2400ms->i2400m);
}


extern int i2400ms_rx_setup(struct i2400ms *);
extern void i2400ms_rx_release(struct i2400ms *);
extern ssize_t __i2400ms_rx_get_size(struct i2400ms *);

extern int i2400ms_tx_setup(struct i2400ms *);
extern void i2400ms_tx_release(struct i2400ms *);
extern void i2400ms_bus_tx_kick(struct i2400m *);

extern ssize_t i2400ms_bus_bm_cmd_send(struct i2400m *,
				       const struct i2400m_bootrom_header *,
				       size_t, int);
extern ssize_t i2400ms_bus_bm_wait_for_ack(struct i2400m *,
					   struct i2400m_bootrom_header *,
					   size_t);
extern void i2400ms_bus_bm_release(struct i2400m *);
extern int i2400ms_bus_bm_setup(struct i2400m *);

#endif /* #ifndef __I2400M_SDIO_H__ */
