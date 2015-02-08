

#ifndef SI4713_H
#define SI4713_H

/* The SI4713 I2C sensor chip has a fixed slave address of 0xc6 or 0x22. */
#define SI4713_I2C_ADDR_BUSEN_HIGH	0x63
#define SI4713_I2C_ADDR_BUSEN_LOW	0x11

struct si4713_platform_data {
	/* Set power state, zero is off, non-zero is on. */
	int (*set_power)(int power);
};

struct si4713_rnl {
	__u32 index;		/* modulator index */
	__u32 frequency;	/* frequency to peform rnl measurement */
	__s32 rnl;		/* result of measurement in dBuV */
	__u32 reserved[4];	/* drivers and apps must init this to 0 */
};

#define SI4713_IOC_MEASURE_RNL	_IOWR('V', BASE_VIDIOC_PRIVATE + 0, \
						struct si4713_rnl)

#endif /* ifndef SI4713_H*/
