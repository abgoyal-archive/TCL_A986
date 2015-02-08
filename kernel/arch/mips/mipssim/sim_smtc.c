
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/cpumask.h>
#include <linux/interrupt.h>
#include <linux/smp.h>

#include <asm/atomic.h>
#include <asm/cpu.h>
#include <asm/processor.h>
#include <asm/system.h>
#include <asm/mmu_context.h>
#include <asm/smtc_ipi.h>

/* VPE/SMP Prototype implements platform interfaces directly */


static void ssmtc_send_ipi_single(int cpu, unsigned int action)
{
	smtc_send_ipi(cpu, LINUX_SMP_IPI, action);
	/* "CPU" may be TC of same VPE, VPE of same CPU, or different CPU */
}

static inline void ssmtc_send_ipi_mask(const struct cpumask *mask,
				       unsigned int action)
{
	unsigned int i;

	for_each_cpu(i, mask)
		ssmtc_send_ipi_single(i, action);
}

static void __cpuinit ssmtc_init_secondary(void)
{
	void smtc_init_secondary(void);

	smtc_init_secondary();
}

static void __cpuinit ssmtc_smp_finish(void)
{
	smtc_smp_finish();
}

static void ssmtc_cpus_done(void)
{
}

static void __cpuinit ssmtc_boot_secondary(int cpu, struct task_struct *idle)
{
	smtc_boot_secondary(cpu, idle);
}

static void __init ssmtc_smp_setup(void)
{
	if (read_c0_config3() & (1 << 2))
		mipsmt_build_cpu_map(0);
}

static void ssmtc_prepare_cpus(unsigned int max_cpus)
{
	/*
	 * As noted above, we can assume a single CPU for now
	 * but it may be multithreaded.
	 */

	if (read_c0_config3() & (1 << 2)) {
		mipsmt_prepare_cpus();
	}
}

struct plat_smp_ops ssmtc_smp_ops = {
	.send_ipi_single	= ssmtc_send_ipi_single,
	.send_ipi_mask		= ssmtc_send_ipi_mask,
	.init_secondary		= ssmtc_init_secondary,
	.smp_finish		= ssmtc_smp_finish,
	.cpus_done		= ssmtc_cpus_done,
	.boot_secondary		= ssmtc_boot_secondary,
	.smp_setup		= ssmtc_smp_setup,
	.prepare_cpus		= ssmtc_prepare_cpus,
};
