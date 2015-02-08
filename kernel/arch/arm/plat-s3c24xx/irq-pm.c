

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/sysdev.h>

#include <plat/cpu.h>
#include <plat/pm.h>
#include <plat/irq.h>

/* state for IRQs over sleep */


unsigned long s3c_irqwake_intallow	= 1L << (IRQ_RTC - IRQ_EINT0) | 0xfL;
unsigned long s3c_irqwake_eintallow	= 0x0000fff0L;

int s3c_irq_wake(unsigned int irqno, unsigned int state)
{
	unsigned long irqbit = 1 << (irqno - IRQ_EINT0);

	if (!(s3c_irqwake_intallow & irqbit))
		return -ENOENT;

	printk(KERN_INFO "wake %s for irq %d\n",
	       state ? "enabled" : "disabled", irqno);

	if (!state)
		s3c_irqwake_intmask |= irqbit;
	else
		s3c_irqwake_intmask &= ~irqbit;

	return 0;
}

static struct sleep_save irq_save[] = {
	SAVE_ITEM(S3C2410_INTMSK),
	SAVE_ITEM(S3C2410_INTSUBMSK),
};


static unsigned long save_extint[3];
static unsigned long save_eintflt[4];
static unsigned long save_eintmask;

int s3c24xx_irq_suspend(struct sys_device *dev, pm_message_t state)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(save_extint); i++)
		save_extint[i] = __raw_readl(S3C24XX_EXTINT0 + (i*4));

	for (i = 0; i < ARRAY_SIZE(save_eintflt); i++)
		save_eintflt[i] = __raw_readl(S3C24XX_EINFLT0 + (i*4));

	s3c_pm_do_save(irq_save, ARRAY_SIZE(irq_save));
	save_eintmask = __raw_readl(S3C24XX_EINTMASK);

	return 0;
}

int s3c24xx_irq_resume(struct sys_device *dev)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(save_extint); i++)
		__raw_writel(save_extint[i], S3C24XX_EXTINT0 + (i*4));

	for (i = 0; i < ARRAY_SIZE(save_eintflt); i++)
		__raw_writel(save_eintflt[i], S3C24XX_EINFLT0 + (i*4));

	s3c_pm_do_restore(irq_save, ARRAY_SIZE(irq_save));
	__raw_writel(save_eintmask, S3C24XX_EINTMASK);

	return 0;
}
