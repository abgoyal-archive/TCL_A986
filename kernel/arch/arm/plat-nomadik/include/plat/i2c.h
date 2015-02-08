
#ifndef __PLAT_I2C_H
#define __PLAT_I2C_H

enum i2c_freq_mode {
	I2C_FREQ_MODE_STANDARD,		/* up to 100 Kb/s */
	I2C_FREQ_MODE_FAST,		/* up to 400 Kb/s */
	I2C_FREQ_MODE_FAST_PLUS,	/* up to 1 Mb/s */
	I2C_FREQ_MODE_HIGH_SPEED	/* up to 3.4 Mb/s */
};

struct nmk_i2c_controller {
	unsigned long	clk_freq;
	unsigned short	slsu;
	unsigned char 	tft;
	unsigned char 	rft;
	enum i2c_freq_mode	sm;
};

#endif	/* __PLAT_I2C_H */
