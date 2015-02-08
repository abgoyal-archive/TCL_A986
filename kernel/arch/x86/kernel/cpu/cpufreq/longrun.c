

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/timex.h>

#include <asm/msr.h>
#include <asm/processor.h>

#define dprintk(msg...) cpufreq_debug_printk(CPUFREQ_DEBUG_DRIVER, \
		"longrun", msg)

static struct cpufreq_driver	longrun_driver;

static unsigned int longrun_low_freq, longrun_high_freq;


static void __init longrun_get_policy(struct cpufreq_policy *policy)
{
	u32 msr_lo, msr_hi;

	rdmsr(MSR_TMTA_LONGRUN_FLAGS, msr_lo, msr_hi);
	dprintk("longrun flags are %x - %x\n", msr_lo, msr_hi);
	if (msr_lo & 0x01)
		policy->policy = CPUFREQ_POLICY_PERFORMANCE;
	else
		policy->policy = CPUFREQ_POLICY_POWERSAVE;

	rdmsr(MSR_TMTA_LONGRUN_CTRL, msr_lo, msr_hi);
	dprintk("longrun ctrl is %x - %x\n", msr_lo, msr_hi);
	msr_lo &= 0x0000007F;
	msr_hi &= 0x0000007F;

	if (longrun_high_freq <= longrun_low_freq) {
		/* Assume degenerate Longrun table */
		policy->min = policy->max = longrun_high_freq;
	} else {
		policy->min = longrun_low_freq + msr_lo *
			((longrun_high_freq - longrun_low_freq) / 100);
		policy->max = longrun_low_freq + msr_hi *
			((longrun_high_freq - longrun_low_freq) / 100);
	}
	policy->cpu = 0;
}


static int longrun_set_policy(struct cpufreq_policy *policy)
{
	u32 msr_lo, msr_hi;
	u32 pctg_lo, pctg_hi;

	if (!policy)
		return -EINVAL;

	if (longrun_high_freq <= longrun_low_freq) {
		/* Assume degenerate Longrun table */
		pctg_lo = pctg_hi = 100;
	} else {
		pctg_lo = (policy->min - longrun_low_freq) /
			((longrun_high_freq - longrun_low_freq) / 100);
		pctg_hi = (policy->max - longrun_low_freq) /
			((longrun_high_freq - longrun_low_freq) / 100);
	}

	if (pctg_hi > 100)
		pctg_hi = 100;
	if (pctg_lo > pctg_hi)
		pctg_lo = pctg_hi;

	/* performance or economy mode */
	rdmsr(MSR_TMTA_LONGRUN_FLAGS, msr_lo, msr_hi);
	msr_lo &= 0xFFFFFFFE;
	switch (policy->policy) {
	case CPUFREQ_POLICY_PERFORMANCE:
		msr_lo |= 0x00000001;
		break;
	case CPUFREQ_POLICY_POWERSAVE:
		break;
	}
	wrmsr(MSR_TMTA_LONGRUN_FLAGS, msr_lo, msr_hi);

	/* lower and upper boundary */
	rdmsr(MSR_TMTA_LONGRUN_CTRL, msr_lo, msr_hi);
	msr_lo &= 0xFFFFFF80;
	msr_hi &= 0xFFFFFF80;
	msr_lo |= pctg_lo;
	msr_hi |= pctg_hi;
	wrmsr(MSR_TMTA_LONGRUN_CTRL, msr_lo, msr_hi);

	return 0;
}


static int longrun_verify_policy(struct cpufreq_policy *policy)
{
	if (!policy)
		return -EINVAL;

	policy->cpu = 0;
	cpufreq_verify_within_limits(policy,
		policy->cpuinfo.min_freq,
		policy->cpuinfo.max_freq);

	if ((policy->policy != CPUFREQ_POLICY_POWERSAVE) &&
	    (policy->policy != CPUFREQ_POLICY_PERFORMANCE))
		return -EINVAL;

	return 0;
}

static unsigned int longrun_get(unsigned int cpu)
{
	u32 eax, ebx, ecx, edx;

	if (cpu)
		return 0;

	cpuid(0x80860007, &eax, &ebx, &ecx, &edx);
	dprintk("cpuid eax is %u\n", eax);

	return eax * 1000;
}

static unsigned int __init longrun_determine_freqs(unsigned int *low_freq,
						   unsigned int *high_freq)
{
	u32 msr_lo, msr_hi;
	u32 save_lo, save_hi;
	u32 eax, ebx, ecx, edx;
	u32 try_hi;
	struct cpuinfo_x86 *c = &cpu_data(0);

	if (!low_freq || !high_freq)
		return -EINVAL;

	if (cpu_has(c, X86_FEATURE_LRTI)) {
		/* if the LongRun Table Interface is present, the
		 * detection is a bit easier:
		 * For minimum frequency, read out the maximum
		 * level (msr_hi), write that into "currently
		 * selected level", and read out the frequency.
		 * For maximum frequency, read out level zero.
		 */
		/* minimum */
		rdmsr(MSR_TMTA_LRTI_READOUT, msr_lo, msr_hi);
		wrmsr(MSR_TMTA_LRTI_READOUT, msr_hi, msr_hi);
		rdmsr(MSR_TMTA_LRTI_VOLT_MHZ, msr_lo, msr_hi);
		*low_freq = msr_lo * 1000; /* to kHz */

		/* maximum */
		wrmsr(MSR_TMTA_LRTI_READOUT, 0, msr_hi);
		rdmsr(MSR_TMTA_LRTI_VOLT_MHZ, msr_lo, msr_hi);
		*high_freq = msr_lo * 1000; /* to kHz */

		dprintk("longrun table interface told %u - %u kHz\n",
				*low_freq, *high_freq);

		if (*low_freq > *high_freq)
			*low_freq = *high_freq;
		return 0;
	}

	/* set the upper border to the value determined during TSC init */
	*high_freq = (cpu_khz / 1000);
	*high_freq = *high_freq * 1000;
	dprintk("high frequency is %u kHz\n", *high_freq);

	/* get current borders */
	rdmsr(MSR_TMTA_LONGRUN_CTRL, msr_lo, msr_hi);
	save_lo = msr_lo & 0x0000007F;
	save_hi = msr_hi & 0x0000007F;

	/* if current perf_pctg is larger than 90%, we need to decrease the
	 * upper limit to make the calculation more accurate.
	 */
	cpuid(0x80860007, &eax, &ebx, &ecx, &edx);
	/* try decreasing in 10% steps, some processors react only
	 * on some barrier values */
	for (try_hi = 80; try_hi > 0 && ecx > 90; try_hi -= 10) {
		/* set to 0 to try_hi perf_pctg */
		msr_lo &= 0xFFFFFF80;
		msr_hi &= 0xFFFFFF80;
		msr_hi |= try_hi;
		wrmsr(MSR_TMTA_LONGRUN_CTRL, msr_lo, msr_hi);

		/* read out current core MHz and current perf_pctg */
		cpuid(0x80860007, &eax, &ebx, &ecx, &edx);

		/* restore values */
		wrmsr(MSR_TMTA_LONGRUN_CTRL, save_lo, save_hi);
	}
	dprintk("percentage is %u %%, freq is %u MHz\n", ecx, eax);

	/* performance_pctg = (current_freq - low_freq)/(high_freq - low_freq)
	 * eqals
	 * low_freq * (1 - perf_pctg) = (cur_freq - high_freq * perf_pctg)
	 *
	 * high_freq * perf_pctg is stored tempoarily into "ebx".
	 */
	ebx = (((cpu_khz / 1000) * ecx) / 100); /* to MHz */

	if ((ecx > 95) || (ecx == 0) || (eax < ebx))
		return -EIO;

	edx = ((eax - ebx) * 100) / (100 - ecx);
	*low_freq = edx * 1000; /* back to kHz */

	dprintk("low frequency is %u kHz\n", *low_freq);

	if (*low_freq > *high_freq)
		*low_freq = *high_freq;

	return 0;
}


static int __init longrun_cpu_init(struct cpufreq_policy *policy)
{
	int result = 0;

	/* capability check */
	if (policy->cpu != 0)
		return -ENODEV;

	/* detect low and high frequency */
	result = longrun_determine_freqs(&longrun_low_freq, &longrun_high_freq);
	if (result)
		return result;

	/* cpuinfo and default policy values */
	policy->cpuinfo.min_freq = longrun_low_freq;
	policy->cpuinfo.max_freq = longrun_high_freq;
	policy->cpuinfo.transition_latency = CPUFREQ_ETERNAL;
	longrun_get_policy(policy);

	return 0;
}


static struct cpufreq_driver longrun_driver = {
	.flags		= CPUFREQ_CONST_LOOPS,
	.verify		= longrun_verify_policy,
	.setpolicy	= longrun_set_policy,
	.get		= longrun_get,
	.init		= longrun_cpu_init,
	.name		= "longrun",
	.owner		= THIS_MODULE,
};


static int __init longrun_init(void)
{
	struct cpuinfo_x86 *c = &cpu_data(0);

	if (c->x86_vendor != X86_VENDOR_TRANSMETA ||
	    !cpu_has(c, X86_FEATURE_LONGRUN))
		return -ENODEV;

	return cpufreq_register_driver(&longrun_driver);
}


static void __exit longrun_exit(void)
{
	cpufreq_unregister_driver(&longrun_driver);
}


MODULE_AUTHOR("Dominik Brodowski <linux@brodo.de>");
MODULE_DESCRIPTION("LongRun driver for Transmeta Crusoe and "
		"Efficeon processors.");
MODULE_LICENSE("GPL");

module_init(longrun_init);
module_exit(longrun_exit);
