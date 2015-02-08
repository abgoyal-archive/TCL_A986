

#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <asm/irq_cpu.h>

void plat_irq_dispatch(void)
{
	u32 cause;

	cause = read_c0_cause() & read_c0_status() & CAUSEF_IP;

	clear_c0_status(cause);

	if (cause & CAUSEF_IP7)
		do_IRQ(7);
	if (cause & CAUSEF_IP2)
		do_IRQ(2);
	if (cause & CAUSEF_IP3)
		do_IRQ(3);
	if (cause & CAUSEF_IP4)
		do_IRQ(4);
	if (cause & CAUSEF_IP5)
		do_IRQ(5);
	if (cause & CAUSEF_IP6)
		do_IRQ(6);
}

void __init arch_init_irq(void)
{
	mips_cpu_irq_init();
}
