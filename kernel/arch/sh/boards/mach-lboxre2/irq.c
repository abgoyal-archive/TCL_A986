
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <mach/lboxre2.h>

void __init init_lboxre2_IRQ(void)
{
	make_imask_irq(IRQ_CF1);
	make_imask_irq(IRQ_CF0);
	make_imask_irq(IRQ_INTD);
	make_imask_irq(IRQ_ETH1);
	make_imask_irq(IRQ_ETH0);
	make_imask_irq(IRQ_INTA);
}
