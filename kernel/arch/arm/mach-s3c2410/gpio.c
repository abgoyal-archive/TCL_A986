

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/io.h>

#include <mach/hardware.h>
#include <mach/gpio-fns.h>
#include <asm/irq.h>

#include <mach/regs-gpio.h>

int s3c2410_gpio_irqfilter(unsigned int pin, unsigned int on,
			   unsigned int config)
{
	void __iomem *reg = S3C24XX_EINFLT0;
	unsigned long flags;
	unsigned long val;

	if (pin < S3C2410_GPG(8) || pin > S3C2410_GPG(15))
		return -EINVAL;

	config &= 0xff;

	pin -= S3C2410_GPG(8);
	reg += pin & ~3;

	local_irq_save(flags);

	/* update filter width and clock source */

	val = __raw_readl(reg);
	val &= ~(0xff << ((pin & 3) * 8));
	val |= config << ((pin & 3) * 8);
	__raw_writel(val, reg);

	/* update filter enable */

	val = __raw_readl(S3C24XX_EXTINT2);
	val &= ~(1 << ((pin * 4) + 3));
	val |= on << ((pin * 4) + 3);
	__raw_writel(val, S3C24XX_EXTINT2);

	local_irq_restore(flags);

	return 0;
}

EXPORT_SYMBOL(s3c2410_gpio_irqfilter);
