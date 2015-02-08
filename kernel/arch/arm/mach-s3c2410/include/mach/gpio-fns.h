

#ifndef __MACH_GPIO_FNS_H
#define __MACH_GPIO_FNS_H __FILE__


#include <plat/gpio-cfg.h>

static inline void s3c2410_gpio_cfgpin(unsigned int pin, unsigned int cfg)
{
	/* 1:1 mapping between cfgpin and setcfg calls at the moment */
	s3c_gpio_cfgpin(pin, cfg);
}


extern unsigned int s3c2410_gpio_getcfg(unsigned int pin);


extern int s3c2410_gpio_getirq(unsigned int pin);

#ifdef CONFIG_CPU_S3C2400

extern int s3c2400_gpio_getirq(unsigned int pin);

#endif /* CONFIG_CPU_S3C2400 */


extern int s3c2410_gpio_irqfilter(unsigned int pin, unsigned int on,
				  unsigned int config);



extern void s3c2410_gpio_pullup(unsigned int pin, unsigned int to);

extern void s3c2410_gpio_setpin(unsigned int pin, unsigned int to);

extern unsigned int s3c2410_gpio_getpin(unsigned int pin);

#endif /* __MACH_GPIO_FNS_H */
