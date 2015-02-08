

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/sysdev.h>

#include <plat/cpu.h>
#include <plat/pm.h>

static int s3c2410_irq_add(struct sys_device *sysdev)
{
	return 0;
}

static struct sysdev_driver s3c2410_irq_driver = {
	.add		= s3c2410_irq_add,
	.suspend	= s3c24xx_irq_suspend,
	.resume		= s3c24xx_irq_resume,
};

static int __init s3c2410_irq_init(void)
{
	return sysdev_driver_register(&s3c2410_sysclass, &s3c2410_irq_driver);
}

arch_initcall(s3c2410_irq_init);

static struct sysdev_driver s3c2410a_irq_driver = {
	.add		= s3c2410_irq_add,
	.suspend	= s3c24xx_irq_suspend,
	.resume		= s3c24xx_irq_resume,
};

static int __init s3c2410a_irq_init(void)
{
	return sysdev_driver_register(&s3c2410a_sysclass, &s3c2410a_irq_driver);
}

arch_initcall(s3c2410a_irq_init);
