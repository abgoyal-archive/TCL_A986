
#include <linux/bitops.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel_stat.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <asm/io.h>

#include <asm/smp.h>

#undef PARISC_IRQ_CR16_COUNTS

extern irqreturn_t timer_interrupt(int, void *);
extern irqreturn_t ipi_interrupt(int, void *);

#define EIEM_MASK(irq)       (1UL<<(CPU_IRQ_MAX - irq))

static volatile unsigned long cpu_eiem = 0;

static DEFINE_PER_CPU(unsigned long, local_ack_eiem) = ~0UL;

static void cpu_disable_irq(unsigned int irq)
{
	unsigned long eirr_bit = EIEM_MASK(irq);

	cpu_eiem &= ~eirr_bit;
	/* Do nothing on the other CPUs.  If they get this interrupt,
	 * The & cpu_eiem in the do_cpu_irq_mask() ensures they won't
	 * handle it, and the set_eiem() at the bottom will ensure it
	 * then gets disabled */
}

static void cpu_enable_irq(unsigned int irq)
{
	unsigned long eirr_bit = EIEM_MASK(irq);

	cpu_eiem |= eirr_bit;

	/* This is just a simple NOP IPI.  But what it does is cause
	 * all the other CPUs to do a set_eiem(cpu_eiem) at the end
	 * of the interrupt handler */
	smp_send_all_nop();
}

static unsigned int cpu_startup_irq(unsigned int irq)
{
	cpu_enable_irq(irq);
	return 0;
}

void no_ack_irq(unsigned int irq) { }
void no_end_irq(unsigned int irq) { }

void cpu_ack_irq(unsigned int irq)
{
	unsigned long mask = EIEM_MASK(irq);
	int cpu = smp_processor_id();

	/* Clear in EIEM so we can no longer process */
	per_cpu(local_ack_eiem, cpu) &= ~mask;

	/* disable the interrupt */
	set_eiem(cpu_eiem & per_cpu(local_ack_eiem, cpu));

	/* and now ack it */
	mtctl(mask, 23);
}

void cpu_end_irq(unsigned int irq)
{
	unsigned long mask = EIEM_MASK(irq);
	int cpu = smp_processor_id();

	/* set it in the eiems---it's no longer in process */
	per_cpu(local_ack_eiem, cpu) |= mask;

	/* enable the interrupt */
	set_eiem(cpu_eiem & per_cpu(local_ack_eiem, cpu));
}

#ifdef CONFIG_SMP
int cpu_check_affinity(unsigned int irq, const struct cpumask *dest)
{
	int cpu_dest;

	/* timer and ipi have to always be received on all CPUs */
	if (CHECK_IRQ_PER_CPU(irq)) {
		/* Bad linux design decision.  The mask has already
		 * been set; we must reset it */
		cpumask_setall(irq_desc[irq].affinity);
		return -EINVAL;
	}

	/* whatever mask they set, we just allow one CPU */
	cpu_dest = first_cpu(*dest);

	return cpu_dest;
}

static int cpu_set_affinity_irq(unsigned int irq, const struct cpumask *dest)
{
	int cpu_dest;

	cpu_dest = cpu_check_affinity(irq, dest);
	if (cpu_dest < 0)
		return -1;

	cpumask_copy(irq_desc[irq].affinity, dest);

	return 0;
}
#endif

static struct irq_chip cpu_interrupt_type = {
	.name		= "CPU",
	.startup	= cpu_startup_irq,
	.shutdown	= cpu_disable_irq,
	.enable		= cpu_enable_irq,
	.disable	= cpu_disable_irq,
	.ack		= cpu_ack_irq,
	.end		= cpu_end_irq,
#ifdef CONFIG_SMP
	.set_affinity	= cpu_set_affinity_irq,
#endif
	/* XXX: Needs to be written.  We managed without it so far, but
	 * we really ought to write it.
	 */
	.retrigger	= NULL,
};

int show_interrupts(struct seq_file *p, void *v)
{
	int i = *(loff_t *) v, j;
	unsigned long flags;

	if (i == 0) {
		seq_puts(p, "    ");
		for_each_online_cpu(j)
			seq_printf(p, "       CPU%d", j);

#ifdef PARISC_IRQ_CR16_COUNTS
		seq_printf(p, " [min/avg/max] (CPU cycle counts)");
#endif
		seq_putc(p, '\n');
	}

	if (i < NR_IRQS) {
		struct irqaction *action;

		raw_spin_lock_irqsave(&irq_desc[i].lock, flags);
		action = irq_desc[i].action;
		if (!action)
			goto skip;
		seq_printf(p, "%3d: ", i);
#ifdef CONFIG_SMP
		for_each_online_cpu(j)
			seq_printf(p, "%10u ", kstat_irqs_cpu(i, j));
#else
		seq_printf(p, "%10u ", kstat_irqs(i));
#endif

		seq_printf(p, " %14s", irq_desc[i].chip->name);
#ifndef PARISC_IRQ_CR16_COUNTS
		seq_printf(p, "  %s", action->name);

		while ((action = action->next))
			seq_printf(p, ", %s", action->name);
#else
		for ( ;action; action = action->next) {
			unsigned int k, avg, min, max;

			min = max = action->cr16_hist[0];

			for (avg = k = 0; k < PARISC_CR16_HIST_SIZE; k++) {
				int hist = action->cr16_hist[k];

				if (hist) {
					avg += hist;
				} else
					break;

				if (hist > max) max = hist;
				if (hist < min) min = hist;
			}

			avg /= k;
			seq_printf(p, " %s[%d/%d/%d]", action->name,
					min,avg,max);
		}
#endif

		seq_putc(p, '\n');
 skip:
		raw_spin_unlock_irqrestore(&irq_desc[i].lock, flags);
	}

	return 0;
}




int cpu_claim_irq(unsigned int irq, struct irq_chip *type, void *data)
{
	if (irq_desc[irq].action)
		return -EBUSY;
	if (irq_desc[irq].chip != &cpu_interrupt_type)
		return -EBUSY;

	if (type) {
		irq_desc[irq].chip = type;
		irq_desc[irq].chip_data = data;
		cpu_interrupt_type.enable(irq);
	}
	return 0;
}

int txn_claim_irq(int irq)
{
	return cpu_claim_irq(irq, NULL, NULL) ? -1 : irq;
}

int txn_alloc_irq(unsigned int bits_wide)
{
	int irq;

	/* never return irq 0 cause that's the interval timer */
	for (irq = CPU_IRQ_BASE + 1; irq <= CPU_IRQ_MAX; irq++) {
		if (cpu_claim_irq(irq, NULL, NULL) < 0)
			continue;
		if ((irq - CPU_IRQ_BASE) >= (1 << bits_wide))
			continue;
		return irq;
	}

	/* unlikely, but be prepared */
	return -1;
}


unsigned long txn_affinity_addr(unsigned int irq, int cpu)
{
#ifdef CONFIG_SMP
	cpumask_copy(irq_desc[irq].affinity, cpumask_of(cpu));
#endif

	return per_cpu(cpu_data, cpu).txn_addr;
}


unsigned long txn_alloc_addr(unsigned int virt_irq)
{
	static int next_cpu = -1;

	next_cpu++; /* assign to "next" CPU we want this bugger on */

	/* validate entry */
	while ((next_cpu < nr_cpu_ids) &&
		(!per_cpu(cpu_data, next_cpu).txn_addr ||
		 !cpu_online(next_cpu)))
		next_cpu++;

	if (next_cpu >= nr_cpu_ids) 
		next_cpu = 0;	/* nothing else, assign monarch */

	return txn_affinity_addr(virt_irq, next_cpu);
}


unsigned int txn_alloc_data(unsigned int virt_irq)
{
	return virt_irq - CPU_IRQ_BASE;
}

static inline int eirr_to_irq(unsigned long eirr)
{
	int bit = fls_long(eirr);
	return (BITS_PER_LONG - bit) + TIMER_IRQ;
}

/* ONLY called from entry.S:intr_extint() */
void do_cpu_irq_mask(struct pt_regs *regs)
{
	struct pt_regs *old_regs;
	unsigned long eirr_val;
	int irq, cpu = smp_processor_id();
#ifdef CONFIG_SMP
	cpumask_t dest;
#endif

	old_regs = set_irq_regs(regs);
	local_irq_disable();
	irq_enter();

	eirr_val = mfctl(23) & cpu_eiem & per_cpu(local_ack_eiem, cpu);
	if (!eirr_val)
		goto set_out;
	irq = eirr_to_irq(eirr_val);

#ifdef CONFIG_SMP
	cpumask_copy(&dest, irq_desc[irq].affinity);
	if (CHECK_IRQ_PER_CPU(irq_desc[irq].status) &&
	    !cpu_isset(smp_processor_id(), dest)) {
		int cpu = first_cpu(dest);

		printk(KERN_DEBUG "redirecting irq %d from CPU %d to %d\n",
		       irq, smp_processor_id(), cpu);
		gsc_writel(irq + CPU_IRQ_BASE,
			   per_cpu(cpu_data, cpu).hpa);
		goto set_out;
	}
#endif
	__do_IRQ(irq);

 out:
	irq_exit();
	set_irq_regs(old_regs);
	return;

 set_out:
	set_eiem(cpu_eiem & per_cpu(local_ack_eiem, cpu));
	goto out;
}

static struct irqaction timer_action = {
	.handler = timer_interrupt,
	.name = "timer",
	.flags = IRQF_DISABLED | IRQF_TIMER | IRQF_PERCPU | IRQF_IRQPOLL,
};

#ifdef CONFIG_SMP
static struct irqaction ipi_action = {
	.handler = ipi_interrupt,
	.name = "IPI",
	.flags = IRQF_DISABLED | IRQF_PERCPU,
};
#endif

static void claim_cpu_irqs(void)
{
	int i;
	for (i = CPU_IRQ_BASE; i <= CPU_IRQ_MAX; i++) {
		irq_desc[i].chip = &cpu_interrupt_type;
	}

	irq_desc[TIMER_IRQ].action = &timer_action;
	irq_desc[TIMER_IRQ].status = IRQ_PER_CPU;
#ifdef CONFIG_SMP
	irq_desc[IPI_IRQ].action = &ipi_action;
	irq_desc[IPI_IRQ].status = IRQ_PER_CPU;
#endif
}

void __init init_IRQ(void)
{
	local_irq_disable();	/* PARANOID - should already be disabled */
	mtctl(~0UL, 23);	/* EIRR : clear all pending external intr */
	claim_cpu_irqs();
#ifdef CONFIG_SMP
	if (!cpu_eiem)
		cpu_eiem = EIEM_MASK(IPI_IRQ) | EIEM_MASK(TIMER_IRQ);
#else
	cpu_eiem = EIEM_MASK(TIMER_IRQ);
#endif
        set_eiem(cpu_eiem);	/* EIEM : enable all external intr */

}
