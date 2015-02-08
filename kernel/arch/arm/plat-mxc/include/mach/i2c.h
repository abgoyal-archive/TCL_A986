

#ifndef __ASM_ARCH_I2C_H_
#define __ASM_ARCH_I2C_H_

struct imxi2c_platform_data {
	int (*init)(struct device *dev);
	void (*exit)(struct device *dev);
	int bitrate;
};

#endif /* __ASM_ARCH_I2C_H_ */
