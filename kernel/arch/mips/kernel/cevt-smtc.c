
#include <linux/clockchips.h>
#include <linux/interrupt.h>
#include <linux/percpu.h>
#include <linux/smp.h>

#include <asm/smtc_ipi.h>
#include <asm/time.h>
#include <asm/cevt-r4k.h>


unsigned long smtc_nexttime[NR_CPUS][NR_CPUS];
static int smtc_nextinvpe[NR_CPUS];


#define MAKEVALID(x) (((x) == 0L) ? 1L : (x))
#define ISVALID(x) ((x) != 0L)


#define IS_SOONER(a, b, reference) \
    (((a) - (unsigned long)(reference)) < ((b) - (unsigned long)(reference)))


#define CATCHUP_INCREMENT 64

static int mips_next_event(unsigned long delta,
				struct clock_event_device *evt)
{
	unsigned long flags;
	unsigned int mtflags;
	unsigned long timestamp, reference, previous;
	unsigned long nextcomp = 0L;
	int vpe = current_cpu_data.vpe_id;
	int cpu = smp_processor_id();
	local_irq_save(flags);
	mtflags = dmt();

	/*
	 * Maintain the per-TC virtual timer
	 * and program the per-VPE shared Count register
	 * as appropriate here...
	 */
	reference = (unsigned long)read_c0_count();
	timestamp = MAKEVALID(reference + delta);
	/*
	 * To really model the clock, we have to catch the case
	 * where the current next-in-VPE timestamp is the old
	 * timestamp for the calling CPE, but the new value is
	 * in fact later.  In that case, we have to do a full
	 * scan and discover the new next-in-VPE CPU id and
	 * timestamp.
	 */
	previous = smtc_nexttime[vpe][cpu];
	if (cpu == smtc_nextinvpe[vpe] && ISVALID(previous)
	    && IS_SOONER(previous, timestamp, reference)) {
		int i;
		int soonest = cpu;

		/*
		 * Update timestamp array here, so that new
		 * value gets considered along with those of
		 * other virtual CPUs on the VPE.
		 */
		smtc_nexttime[vpe][cpu] = timestamp;
		for_each_online_cpu(i) {
			if (ISVALID(smtc_nexttime[vpe][i])
			    && IS_SOONER(smtc_nexttime[vpe][i],
				smtc_nexttime[vpe][soonest], reference)) {
				    soonest = i;
			}
		}
		smtc_nextinvpe[vpe] = soonest;
		nextcomp = smtc_nexttime[vpe][soonest];
	/*
	 * Otherwise, we don't have to process the whole array rank,
	 * we just have to see if the event horizon has gotten closer.
	 */
	} else {
		if (!ISVALID(smtc_nexttime[vpe][smtc_nextinvpe[vpe]]) ||
		    IS_SOONER(timestamp,
			smtc_nexttime[vpe][smtc_nextinvpe[vpe]], reference)) {
			    smtc_nextinvpe[vpe] = cpu;
			    nextcomp = timestamp;
		}
		/*
		 * Since next-in-VPE may me the same as the executing
		 * virtual CPU, we update the array *after* checking
		 * its value.
		 */
		smtc_nexttime[vpe][cpu] = timestamp;
	}

	/*
	 * It may be that, in fact, we don't need to update Compare,
	 * but if we do, we want to make sure we didn't fall into
	 * a crack just behind Count.
	 */
	if (ISVALID(nextcomp)) {
		write_c0_compare(nextcomp);
		ehb();
		/*
		 * We never return an error, we just make sure
		 * that we trigger the handlers as quickly as
		 * we can if we fell behind.
		 */
		while ((nextcomp - (unsigned long)read_c0_count())
			> (unsigned long)LONG_MAX) {
			nextcomp += CATCHUP_INCREMENT;
			write_c0_compare(nextcomp);
			ehb();
		}
	}
	emt(mtflags);
	local_irq_restore(flags);
	return 0;
}


void smtc_distribute_timer(int vpe)
{
	unsigned long flags;
	unsigned int mtflags;
	int cpu;
	struct clock_event_device *cd;
	unsigned long nextstamp;
	unsigned long reference;


repeat:
	nextstamp = 0L;
	for_each_online_cpu(cpu) {
	    /*
	     * Find virtual CPUs within the current VPE who have
	     * unserviced timer requests whose time is now past.
	     */
	    local_irq_save(flags);
	    mtflags = dmt();
	    if (cpu_data[cpu].vpe_id == vpe &&
		ISVALID(smtc_nexttime[vpe][cpu])) {
		reference = (unsigned long)read_c0_count();
		if ((smtc_nexttime[vpe][cpu] - reference)
			 > (unsigned long)LONG_MAX) {
			    smtc_nexttime[vpe][cpu] = 0L;
			    emt(mtflags);
			    local_irq_restore(flags);
			    /*
			     * We don't send IPIs to ourself.
			     */
			    if (cpu != smp_processor_id()) {
				smtc_send_ipi(cpu, SMTC_CLOCK_TICK, 0);
			    } else {
				cd = &per_cpu(mips_clockevent_device, cpu);
				cd->event_handler(cd);
			    }
		} else {
			/* Local to VPE but Valid Time not yet reached. */
			if (!ISVALID(nextstamp) ||
			    IS_SOONER(smtc_nexttime[vpe][cpu], nextstamp,
			    reference)) {
				smtc_nextinvpe[vpe] = cpu;
				nextstamp = smtc_nexttime[vpe][cpu];
			}
			emt(mtflags);
			local_irq_restore(flags);
		}
	    } else {
		emt(mtflags);
		local_irq_restore(flags);

	    }
	}
	/* Reprogram for interrupt at next soonest timestamp for VPE */
	if (ISVALID(nextstamp)) {
		write_c0_compare(nextstamp);
		ehb();
		if ((nextstamp - (unsigned long)read_c0_count())
			> (unsigned long)LONG_MAX)
				goto repeat;
	}
}


irqreturn_t c0_compare_interrupt(int irq, void *dev_id)
{
	int cpu = smp_processor_id();

	/* If we're running SMTC, we've got MIPS MT and therefore MIPS32R2 */
	handle_perf_irq(1);

	if (read_c0_cause() & (1 << 30)) {
		/* Clear Count/Compare Interrupt */
		write_c0_compare(read_c0_compare());
		smtc_distribute_timer(cpu_data[cpu].vpe_id);
	}
	return IRQ_HANDLED;
}


int __cpuinit smtc_clockevent_init(void)
{
	uint64_t mips_freq = mips_hpt_frequency;
	unsigned int cpu = smp_processor_id();
	struct clock_event_device *cd;
	unsigned int irq;
	int i;
	int j;

	if (!cpu_has_counter || !mips_hpt_frequency)
		return -ENXIO;
	if (cpu == 0) {
		for (i = 0; i < num_possible_cpus(); i++) {
			smtc_nextinvpe[i] = 0;
			for (j = 0; j < num_possible_cpus(); j++)
				smtc_nexttime[i][j] = 0L;
		}
		/*
		 * SMTC also can't have the usablility test
		 * run by secondary TCs once Compare is in use.
		 */
		if (!c0_compare_int_usable())
			return -ENXIO;
	}

	/*
	 * With vectored interrupts things are getting platform specific.
	 * get_c0_compare_int is a hook to allow a platform to return the
	 * interrupt number of it's liking.
	 */
	irq = MIPS_CPU_IRQ_BASE + cp0_compare_irq;
	if (get_c0_compare_int)
		irq = get_c0_compare_int();

	cd = &per_cpu(mips_clockevent_device, cpu);

	cd->name		= "MIPS";
	cd->features		= CLOCK_EVT_FEAT_ONESHOT;

	/* Calculate the min / max delta */
	cd->mult	= div_sc((unsigned long) mips_freq, NSEC_PER_SEC, 32);
	cd->shift		= 32;
	cd->max_delta_ns	= clockevent_delta2ns(0x7fffffff, cd);
	cd->min_delta_ns	= clockevent_delta2ns(0x300, cd);

	cd->rating		= 300;
	cd->irq			= irq;
	cd->cpumask		= cpumask_of(cpu);
	cd->set_next_event	= mips_next_event;
	cd->set_mode		= mips_set_clock_mode;
	cd->event_handler	= mips_event_handler;

	clockevents_register_device(cd);

	/*
	 * On SMTC we only want to do the data structure
	 * initialization and IRQ setup once.
	 */
	if (cpu)
		return 0;
	/*
	 * And we need the hwmask associated with the c0_compare
	 * vector to be initialized.
	 */
	irq_hwmask[irq] = (0x100 << cp0_compare_irq);
	if (cp0_timer_irq_installed)
		return 0;

	cp0_timer_irq_installed = 1;

	setup_irq(irq, &c0_compare_irqaction);

	return 0;
}
