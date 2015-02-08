

#ifndef __I2C_PNX_H__
#define __I2C_PNX_H__

struct platform_device;
struct clk;

struct i2c_pnx_mif {
	int			ret;		/* Return value */
	int			mode;		/* Interface mode */
	struct completion	complete;	/* I/O completion */
	struct timer_list	timer;		/* Timeout */
	u8 *			buf;		/* Data buffer */
	int			len;		/* Length of data buffer */
};

struct i2c_pnx_algo_data {
	void __iomem		*ioaddr;
	struct i2c_pnx_mif	mif;
	int			last;
	struct clk		*clk;
	struct i2c_pnx_data	*i2c_pnx;
	struct i2c_adapter	adapter;
};

struct i2c_pnx_data {
	const char *name;
	u32 base;
	int irq;
};

#endif /* __I2C_PNX_H__ */
