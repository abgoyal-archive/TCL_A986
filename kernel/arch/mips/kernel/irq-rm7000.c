
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>

#include <asm/irq_cpu.h>
#include <asm/mipsregs.h>
#include <asm/system.h>

static inline void unmask_rm7k_irq(unsigned int irq)
{
	set_c0_intcontrol(0x100 << (irq - RM7K_CPU_IRQ_BASE));
}

static inline void mask_rm7k_irq(unsigned int irq)
{
	clear_c0_intcontrol(0x100 << (irq - RM7K_CPU_IRQ_BASE));
}

static struct irq_chip rm7k_irq_controller = {
	.name = "RM7000",
	.ack = mask_rm7k_irq,
	.mask = mask_rm7k_irq,
	.mask_ack = mask_rm7k_irq,
	.unmask = unmask_rm7k_irq,
	.eoi	= unmask_rm7k_irq
};

void __init rm7k_cpu_irq_init(void)
{
	int base = RM7K_CPU_IRQ_BASE;
	int i;

	clear_c0_intcontrol(0x00000f00);		/* Mask all */

	for (i = base; i < base + 4; i++)
		set_irq_chip_and_handler(i, &rm7k_irq_controller,
					 handle_percpu_irq);
}
