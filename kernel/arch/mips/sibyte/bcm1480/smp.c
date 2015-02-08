

#include <linux/init.h>
#include <linux/delay.h>
#include <linux/smp.h>
#include <linux/kernel_stat.h>

#include <asm/mmu_context.h>
#include <asm/io.h>
#include <asm/fw/cfe/cfe_api.h>
#include <asm/sibyte/sb1250.h>
#include <asm/sibyte/bcm1480_regs.h>
#include <asm/sibyte/bcm1480_int.h>

extern void smp_call_function_interrupt(void);


static void *mailbox_0_set_regs[] = {
	IOADDR(A_BCM1480_IMR_CPU0_BASE + R_BCM1480_IMR_MAILBOX_0_SET_CPU),
	IOADDR(A_BCM1480_IMR_CPU1_BASE + R_BCM1480_IMR_MAILBOX_0_SET_CPU),
	IOADDR(A_BCM1480_IMR_CPU2_BASE + R_BCM1480_IMR_MAILBOX_0_SET_CPU),
	IOADDR(A_BCM1480_IMR_CPU3_BASE + R_BCM1480_IMR_MAILBOX_0_SET_CPU),
};

static void *mailbox_0_clear_regs[] = {
	IOADDR(A_BCM1480_IMR_CPU0_BASE + R_BCM1480_IMR_MAILBOX_0_CLR_CPU),
	IOADDR(A_BCM1480_IMR_CPU1_BASE + R_BCM1480_IMR_MAILBOX_0_CLR_CPU),
	IOADDR(A_BCM1480_IMR_CPU2_BASE + R_BCM1480_IMR_MAILBOX_0_CLR_CPU),
	IOADDR(A_BCM1480_IMR_CPU3_BASE + R_BCM1480_IMR_MAILBOX_0_CLR_CPU),
};

static void *mailbox_0_regs[] = {
	IOADDR(A_BCM1480_IMR_CPU0_BASE + R_BCM1480_IMR_MAILBOX_0_CPU),
	IOADDR(A_BCM1480_IMR_CPU1_BASE + R_BCM1480_IMR_MAILBOX_0_CPU),
	IOADDR(A_BCM1480_IMR_CPU2_BASE + R_BCM1480_IMR_MAILBOX_0_CPU),
	IOADDR(A_BCM1480_IMR_CPU3_BASE + R_BCM1480_IMR_MAILBOX_0_CPU),
};

void __cpuinit bcm1480_smp_init(void)
{
	unsigned int imask = STATUSF_IP4 | STATUSF_IP3 | STATUSF_IP2 |
		STATUSF_IP1 | STATUSF_IP0;

	/* Set interrupt mask, but don't enable */
	change_c0_status(ST0_IM, imask);
}


static void bcm1480_send_ipi_single(int cpu, unsigned int action)
{
	__raw_writeq((((u64)action)<< 48), mailbox_0_set_regs[cpu]);
}

static void bcm1480_send_ipi_mask(const struct cpumask *mask,
				  unsigned int action)
{
	unsigned int i;

	for_each_cpu(i, mask)
		bcm1480_send_ipi_single(i, action);
}

static void __cpuinit bcm1480_init_secondary(void)
{
	extern void bcm1480_smp_init(void);

	bcm1480_smp_init();
}

static void __cpuinit bcm1480_smp_finish(void)
{
	extern void sb1480_clockevent_init(void);

	sb1480_clockevent_init();
	local_irq_enable();
}

static void bcm1480_cpus_done(void)
{
}

static void __cpuinit bcm1480_boot_secondary(int cpu, struct task_struct *idle)
{
	int retval;

	retval = cfe_cpu_start(cpu_logical_map(cpu), &smp_bootstrap,
			       __KSTK_TOS(idle),
			       (unsigned long)task_thread_info(idle), 0);
	if (retval != 0)
		printk("cfe_start_cpu(%i) returned %i\n" , cpu, retval);
}

static void __init bcm1480_smp_setup(void)
{
	int i, num;

	cpus_clear(cpu_possible_map);
	cpu_set(0, cpu_possible_map);
	__cpu_number_map[0] = 0;
	__cpu_logical_map[0] = 0;

	for (i = 1, num = 0; i < NR_CPUS; i++) {
		if (cfe_cpu_stop(i) == 0) {
			cpu_set(i, cpu_possible_map);
			__cpu_number_map[i] = ++num;
			__cpu_logical_map[num] = i;
		}
	}
	printk(KERN_INFO "Detected %i available secondary CPU(s)\n", num);
}

static void __init bcm1480_prepare_cpus(unsigned int max_cpus)
{
}

struct plat_smp_ops bcm1480_smp_ops = {
	.send_ipi_single	= bcm1480_send_ipi_single,
	.send_ipi_mask		= bcm1480_send_ipi_mask,
	.init_secondary		= bcm1480_init_secondary,
	.smp_finish		= bcm1480_smp_finish,
	.cpus_done		= bcm1480_cpus_done,
	.boot_secondary		= bcm1480_boot_secondary,
	.smp_setup		= bcm1480_smp_setup,
	.prepare_cpus		= bcm1480_prepare_cpus,
};

void bcm1480_mailbox_interrupt(void)
{
	int cpu = smp_processor_id();
	int irq = K_BCM1480_INT_MBOX_0_0;
	unsigned int action;

	kstat_incr_irqs_this_cpu(irq, irq_to_desc(irq));
	/* Load the mailbox register to figure out what we're supposed to do */
	action = (__raw_readq(mailbox_0_regs[cpu]) >> 48) & 0xffff;

	/* Clear the mailbox to clear the interrupt */
	__raw_writeq(((u64)action)<<48, mailbox_0_clear_regs[cpu]);

	/*
	 * Nothing to do for SMP_RESCHEDULE_YOURSELF; returning from the
	 * interrupt will do the reschedule for us
	 */

	if (action & SMP_CALL_FUNCTION)
		smp_call_function_interrupt();
}
