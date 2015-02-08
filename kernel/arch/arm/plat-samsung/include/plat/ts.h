

#ifndef __ASM_ARM_TS_H
#define __ASM_ARM_TS_H

struct s3c2410_ts_mach_info {
       int             delay;
       int             presc;
       int             oversampling_shift;
	void    (*cfg_gpio)(struct platform_device *dev);
};

extern void s3c24xx_ts_set_platdata(struct s3c2410_ts_mach_info *);

/* defined by architecture to configure gpio */
extern void s3c24xx_ts_cfg_gpio(struct platform_device *dev);

#endif /* __ASM_ARM_TS_H */
