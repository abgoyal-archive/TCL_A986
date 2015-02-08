

#undef DEBUG

#include <linux/kernel.h>
#include <linux/smp.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/threads.h>
#include <linux/cpu.h>

#include <asm/irq.h>
#include <asm/smp.h>
#include <asm/machdep.h>
#include <asm/udbg.h>

#include "beat_interrupt.h"

#ifdef DEBUG
#define DBG(fmt...) udbg_printf(fmt)
#else
#define DBG(fmt...)
#endif

/* static cpumask_t of_spin_map; */

static inline int __devinit smp_startup_cpu(unsigned int lcpu)
{
	return 0;
}

static void smp_beatic_message_pass(int target, int msg)
{
	unsigned int i;

	if (target < NR_CPUS) {
		beatic_cause_IPI(target, msg);
	} else {
		for_each_online_cpu(i) {
			if (target == MSG_ALL_BUT_SELF
			    && i == smp_processor_id())
				continue;
			beatic_cause_IPI(i, msg);
		}
	}
}

static int __init smp_beatic_probe(void)
{
	return cpus_weight(cpu_possible_map);
}

static void __devinit smp_beatic_setup_cpu(int cpu)
{
	beatic_setup_cpu(cpu);
}

static void __devinit smp_celleb_kick_cpu(int nr)
{
	BUG_ON(nr < 0 || nr >= NR_CPUS);

	if (!smp_startup_cpu(nr))
		return;
}

static int smp_celleb_cpu_bootable(unsigned int nr)
{
	return 1;
}
static struct smp_ops_t bpa_beatic_smp_ops = {
	.message_pass	= smp_beatic_message_pass,
	.probe		= smp_beatic_probe,
	.kick_cpu	= smp_celleb_kick_cpu,
	.setup_cpu	= smp_beatic_setup_cpu,
	.cpu_bootable	= smp_celleb_cpu_bootable,
};

/* This is called very early */
void __init smp_init_celleb(void)
{
	DBG(" -> smp_init_celleb()\n");

	smp_ops = &bpa_beatic_smp_ops;

	DBG(" <- smp_init_celleb()\n");
}
