

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/cpufreq.h>

#include <asm/cputype.h>

#include <mach/hardware.h>

#include "generic.h"

typedef struct {
	int speed;
	u32 mdcnfg;
	u32 mdcas0;
	u32 mdcas1;
	u32 mdcas2;
} sa1100_dram_regs_t;


static struct cpufreq_driver sa1100_driver;

static sa1100_dram_regs_t sa1100_dram_settings[] =
{
	/* speed,     mdcnfg,     mdcas0,     mdcas1,     mdcas2  clock frequency */
	{  59000, 0x00dc88a3, 0xcccccccf, 0xfffffffc, 0xffffffff }, /*  59.0 MHz */
	{  73700, 0x011490a3, 0xcccccccf, 0xfffffffc, 0xffffffff }, /*  73.7 MHz */
	{  88500, 0x014e90a3, 0xcccccccf, 0xfffffffc, 0xffffffff }, /*  88.5 MHz */
	{ 103200, 0x01889923, 0xcccccccf, 0xfffffffc, 0xffffffff }, /* 103.2 MHz */
	{ 118000, 0x01c29923, 0x9999998f, 0xfffffff9, 0xffffffff }, /* 118.0 MHz */
	{ 132700, 0x01fb2123, 0x9999998f, 0xfffffff9, 0xffffffff }, /* 132.7 MHz */
	{ 147500, 0x02352123, 0x3333330f, 0xfffffff3, 0xffffffff }, /* 147.5 MHz */
	{ 162200, 0x026b29a3, 0x38e38e1f, 0xfff8e38e, 0xffffffff }, /* 162.2 MHz */
	{ 176900, 0x02a329a3, 0x71c71c1f, 0xfff1c71c, 0xffffffff }, /* 176.9 MHz */
	{ 191700, 0x02dd31a3, 0xe38e383f, 0xffe38e38, 0xffffffff }, /* 191.7 MHz */
	{ 206400, 0x03153223, 0xc71c703f, 0xffc71c71, 0xffffffff }, /* 206.4 MHz */
	{ 221200, 0x034fba23, 0xc71c703f, 0xffc71c71, 0xffffffff }, /* 221.2 MHz */
	{ 235900, 0x03853a23, 0xe1e1e07f, 0xe1e1e1e1, 0xffffffe1 }, /* 235.9 MHz */
	{ 250700, 0x03bf3aa3, 0xc3c3c07f, 0xc3c3c3c3, 0xffffffc3 }, /* 250.7 MHz */
	{ 265400, 0x03f7c2a3, 0xc3c3c07f, 0xc3c3c3c3, 0xffffffc3 }, /* 265.4 MHz */
	{ 280200, 0x0431c2a3, 0x878780ff, 0x87878787, 0xffffff87 }, /* 280.2 MHz */
	{ 0, 0, 0, 0, 0 } /* last entry */
};

static void sa1100_update_dram_timings(int current_speed, int new_speed)
{
	sa1100_dram_regs_t *settings = sa1100_dram_settings;

	/* find speed */
	while (settings->speed != 0) {
		if(new_speed == settings->speed)
			break;
		
		settings++;
	}

	if (settings->speed == 0) {
		panic("%s: couldn't find dram setting for speed %d\n",
		      __func__, new_speed);
	}

	/* No risk, no fun: run with interrupts on! */
	if (new_speed > current_speed) {
		/* We're going FASTER, so first relax the memory
		 * timings before changing the core frequency
		 */
		
		/* Half the memory access clock */
		MDCNFG |= MDCNFG_CDB2;

		/* The order of these statements IS important, keep 8
		 * pulses!!
		 */
		MDCAS2 = settings->mdcas2;
		MDCAS1 = settings->mdcas1;
		MDCAS0 = settings->mdcas0;
		MDCNFG = settings->mdcnfg;
	} else {
		/* We're going SLOWER: first decrease the core
		 * frequency and then tighten the memory settings.
		 */

		/* Half the memory access clock */
		MDCNFG |= MDCNFG_CDB2;

		/* The order of these statements IS important, keep 8
		 * pulses!!
		 */
		MDCAS0 = settings->mdcas0;
		MDCAS1 = settings->mdcas1;
		MDCAS2 = settings->mdcas2;
		MDCNFG = settings->mdcnfg;
	}
}

static int sa1100_target(struct cpufreq_policy *policy,
			 unsigned int target_freq,
			 unsigned int relation)
{
	unsigned int cur = sa11x0_getspeed(0);
	unsigned int new_ppcr;

	struct cpufreq_freqs freqs;
	switch(relation){
	case CPUFREQ_RELATION_L:
		new_ppcr = sa11x0_freq_to_ppcr(target_freq);
		if (sa11x0_ppcr_to_freq(new_ppcr) > policy->max)
			new_ppcr--;
		break;
	case CPUFREQ_RELATION_H:
		new_ppcr = sa11x0_freq_to_ppcr(target_freq);
		if ((sa11x0_ppcr_to_freq(new_ppcr) > target_freq) &&
		    (sa11x0_ppcr_to_freq(new_ppcr - 1) >= policy->min))
			new_ppcr--;
		break;
	}

	freqs.old = cur;
	freqs.new = sa11x0_ppcr_to_freq(new_ppcr);
	freqs.cpu = 0;

	cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);

	if (freqs.new > cur)
		sa1100_update_dram_timings(cur, freqs.new);

	PPCR = new_ppcr;

	if (freqs.new < cur)
		sa1100_update_dram_timings(cur, freqs.new);

	cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

	return 0;
}

static int __init sa1100_cpu_init(struct cpufreq_policy *policy)
{
	if (policy->cpu != 0)
		return -EINVAL;
	policy->cur = policy->min = policy->max = sa11x0_getspeed(0);
	policy->cpuinfo.min_freq = 59000;
	policy->cpuinfo.max_freq = 287000;
	policy->cpuinfo.transition_latency = CPUFREQ_ETERNAL;
	return 0;
}

static struct cpufreq_driver sa1100_driver = {
	.flags		= CPUFREQ_STICKY,
	.verify		= sa11x0_verify_speed,
	.target		= sa1100_target,
	.get		= sa11x0_getspeed,
	.init		= sa1100_cpu_init,
	.name		= "sa1100",
};

static int __init sa1100_dram_init(void)
{
	if (cpu_is_sa1100())
		return cpufreq_register_driver(&sa1100_driver);
	else
		return -ENODEV;
}

arch_initcall(sa1100_dram_init);
