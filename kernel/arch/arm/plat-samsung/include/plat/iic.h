

#ifndef __ASM_ARCH_IIC_H
#define __ASM_ARCH_IIC_H __FILE__

#define S3C_IICFLG_FILTER	(1<<0)	/* enable s3c2440 filter */

struct s3c2410_platform_i2c {
	int		bus_num;
	unsigned int	flags;
	unsigned int	slave_addr;
	unsigned long	frequency;
	unsigned int	sda_delay;

	void	(*cfg_gpio)(struct platform_device *dev);
};

extern void s3c_i2c0_set_platdata(struct s3c2410_platform_i2c *i2c);
extern void s3c_i2c1_set_platdata(struct s3c2410_platform_i2c *i2c);
extern void s3c_i2c2_set_platdata(struct s3c2410_platform_i2c *i2c);

/* defined by architecture to configure gpio */
extern void s3c_i2c0_cfg_gpio(struct platform_device *dev);
extern void s3c_i2c1_cfg_gpio(struct platform_device *dev);
extern void s3c_i2c2_cfg_gpio(struct platform_device *dev);

#endif /* __ASM_ARCH_IIC_H */
