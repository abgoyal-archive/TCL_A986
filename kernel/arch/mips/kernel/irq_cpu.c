

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>

#include <asm/irq_cpu.h>
#include <asm/mipsregs.h>
#include <asm/mipsmtregs.h>
#include <asm/system.h>

static inline void unmask_mips_irq(unsigned int irq)
{
	set_c0_status(0x100 << (irq - MIPS_CPU_IRQ_BASE));
	irq_enable_hazard();
}

static inline void mask_mips_irq(unsigned int irq)
{
	clear_c0_status(0x100 << (irq - MIPS_CPU_IRQ_BASE));
	irq_disable_hazard();
}

static struct irq_chip mips_cpu_irq_controller = {
	.name		= "MIPS",
	.ack		= mask_mips_irq,
	.mask		= mask_mips_irq,
	.mask_ack	= mask_mips_irq,
	.unmask		= unmask_mips_irq,
	.eoi		= unmask_mips_irq,
};


#define unmask_mips_mt_irq	unmask_mips_irq
#define mask_mips_mt_irq	mask_mips_irq

static unsigned int mips_mt_cpu_irq_startup(unsigned int irq)
{
	unsigned int vpflags = dvpe();

	clear_c0_cause(0x100 << (irq - MIPS_CPU_IRQ_BASE));
	evpe(vpflags);
	unmask_mips_mt_irq(irq);

	return 0;
}

static void mips_mt_cpu_irq_ack(unsigned int irq)
{
	unsigned int vpflags = dvpe();
	clear_c0_cause(0x100 << (irq - MIPS_CPU_IRQ_BASE));
	evpe(vpflags);
	mask_mips_mt_irq(irq);
}

static struct irq_chip mips_mt_cpu_irq_controller = {
	.name		= "MIPS",
	.startup	= mips_mt_cpu_irq_startup,
	.ack		= mips_mt_cpu_irq_ack,
	.mask		= mask_mips_mt_irq,
	.mask_ack	= mips_mt_cpu_irq_ack,
	.unmask		= unmask_mips_mt_irq,
	.eoi		= unmask_mips_mt_irq,
};

void __init mips_cpu_irq_init(void)
{
	int irq_base = MIPS_CPU_IRQ_BASE;
	int i;

	/* Mask interrupts. */
	clear_c0_status(ST0_IM);
	clear_c0_cause(CAUSEF_IP);

	/*
	 * Only MT is using the software interrupts currently, so we just
	 * leave them uninitialized for other processors.
	 */
	if (cpu_has_mipsmt)
		for (i = irq_base; i < irq_base + 2; i++)
			set_irq_chip_and_handler(i, &mips_mt_cpu_irq_controller,
						 handle_percpu_irq);

	for (i = irq_base + 2; i < irq_base + 8; i++)
		set_irq_chip_and_handler(i, &mips_cpu_irq_controller,
					 handle_percpu_irq);
}
