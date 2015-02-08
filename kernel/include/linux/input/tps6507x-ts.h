

#ifndef __LINUX_I2C_TPS6507X_TS_H
#define __LINUX_I2C_TPS6507X_TS_H

/* Board specific touch screen initial values */
struct touchscreen_init_data {
	int	poll_period;	/* ms */
	int	vref;		/* non-zero to leave vref on */
	__u16	min_pressure;	/* min reading to be treated as a touch */
	__u16	vendor;
	__u16	product;
	__u16	version;
};

#endif /*  __LINUX_I2C_TPS6507X_TS_H */
