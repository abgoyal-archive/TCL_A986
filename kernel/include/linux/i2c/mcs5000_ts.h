

#ifndef __LINUX_MCS5000_TS_H
#define __LINUX_MCS5000_TS_H

/* platform data for the MELFAS MCS-5000 touchscreen driver */
struct mcs5000_ts_platform_data {
	void (*cfg_pin)(void);
	int x_size;
	int y_size;
};

#endif	/* __LINUX_MCS5000_TS_H */
