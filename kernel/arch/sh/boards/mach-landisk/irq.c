
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <mach-landisk/mach/iodata_landisk.h>

static void disable_landisk_irq(unsigned int irq)
{
	unsigned char mask = 0xff ^ (0x01 << (irq - 5));

	__raw_writeb(__raw_readb(PA_IMASK) & mask, PA_IMASK);
}

static void enable_landisk_irq(unsigned int irq)
{
	unsigned char value = (0x01 << (irq - 5));

	__raw_writeb(__raw_readb(PA_IMASK) | value, PA_IMASK);
}

static struct irq_chip landisk_irq_chip __read_mostly = {
	.name		= "LANDISK",
	.mask		= disable_landisk_irq,
	.unmask		= enable_landisk_irq,
	.mask_ack	= disable_landisk_irq,
};

void __init init_landisk_IRQ(void)
{
	int i;

	for (i = 5; i < 14; i++) {
		disable_irq_nosync(i);
		set_irq_chip_and_handler_name(i, &landisk_irq_chip,
					      handle_level_irq, "level");
		enable_landisk_irq(i);
	}
	__raw_writeb(0x00, PA_PWRINT_CLR);
}
