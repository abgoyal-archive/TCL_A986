
#include <linux/kernel_stat.h>
#include <linux/module.h>
#include <linux/signal.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/random.h>
#include <linux/smp.h>
#include <linux/init.h>
#include <linux/seq_file.h>
#include <linux/errno.h>
#include <linux/list.h>
#include <linux/kallsyms.h>
#include <linux/proc_fs.h>

#include <asm/system.h>
#include <asm/mach/irq.h>
#include <asm/mach/time.h>

#ifdef CONFIG_MTK_SCHED_TRACERS
#include <trace/events/sched.h>
extern struct task_struct *mtk_next_task;
// define in: kernel/trace/trace_sched_switch.c
extern int	sched_stopped;
unsigned int g_last_irq = 0;
#endif

#ifndef irq_finish
#define irq_finish(irq) do { } while (0)
#endif

static int irqnr;

void (*init_arch_irq)(void) __initdata = NULL;
unsigned long irq_err_count;

int show_interrupts(struct seq_file *p, void *v)
{
	int i = *(loff_t *) v, cpu;
	struct irqaction * action;
	unsigned long flags;

	if (i == 0) {
		char cpuname[12];

		seq_printf(p, "    ");
		for_each_present_cpu(cpu) {
			sprintf(cpuname, "CPU%d", cpu);
			seq_printf(p, "%10s", cpuname);
		}
		seq_printf(p, "%10s", ":");
		seq_printf(p, "%18s:", "ISR_name");
		seq_printf(p, "%15s:%10s:%10s:%10s", "Total(ns)", "act_counts", "max", "min");
		seq_putc(p, '\n');
	}

	if (i < NR_IRQS) {
		raw_spin_lock_irqsave(&irq_desc[i].lock, flags);
		action = irq_desc[i].action;
		if (!action)
			goto unlock;
		seq_printf(p, "%3d:", i);
		for_each_present_cpu(cpu)
			seq_printf(p, "%10u", kstat_irqs_cpu(i, cpu));
		seq_printf(p, "%10s", irq_desc[i].chip->name ? : ":");
		seq_printf(p, "%18s", action->name);

#ifdef CONFIG_MTPROF_IRQ_DURATION
		seq_printf(p, ":%15llu:%10lu", action->duration, action->count);
		action->count == 0?
		    seq_printf(p, ":%10s:%10s", "0", "0"):
		    seq_printf(p, ":%10llu:%10llu", action->dur_max, action->dur_min);

		for (action = action->next; action; action = action->next){
		    seq_printf(p, "\n%24s%18s", " ", action->name);
		    seq_printf(p, ":%15llu:%10lu", action->duration, action->count);
		    action->count == 0?
			seq_printf(p, ":%10s:%10s", "0", "0"):
			seq_printf(p, ":%10llu:%10llu", action->dur_max, action->dur_min);
		}
#else
		for (action = action->next; action; action = action->next)
		    seq_printf(p, " %s", action->name);
#endif

		seq_putc(p, '\n');
unlock:
		raw_spin_unlock_irqrestore(&irq_desc[i].lock, flags);
	} else if (i == NR_IRQS) {
#ifdef CONFIG_FIQ
		show_fiq_list(p, v);
#endif
#ifdef CONFIG_SMP
		show_ipi_list(p);
		show_local_irqs(p);
#endif
		seq_printf(p, "Err: %10lu\n", irq_err_count);
	}
	return 0;
}

int irq_nr(void)
{
    return irqnr;
}

asmlinkage void __exception asm_do_IRQ(unsigned int irq, struct pt_regs *regs)
{
	struct pt_regs *old_regs = set_irq_regs(regs);

#ifdef CONFIG_MTK_SCHED_TRACERS
	unsigned int last_irq = 0;
	unsigned long flags;
	if (unlikely(!sched_stopped)) {
	    local_irq_save(flags);
	    if(hardirq_count()){
		last_irq = g_last_irq;
		trace_int_nest(last_irq, irq);
		g_last_irq = irq;
	    }
	    else{
		last_irq = irq;
		g_last_irq = irq;
		trace_int_switch(mtk_next_task, irq, 1);
	    }
	    irq_enter();
	    local_irq_restore(flags);
	}else
	    irq_enter();
#else
	irq_enter();
#endif	

	/* save the current IRQ number for kernel panic debugging */
	irqnr = irq;

	/*
	 * Some hardware gives randomly wrong interrupts.  Rather
	 * than crashing, do something sensible.
	 */
	if (unlikely(irq >= NR_IRQS)) {
		if (printk_ratelimit())
			printk(KERN_WARNING "Bad IRQ%u\n", irq);
		ack_bad_irq(irq);
	} else {
		generic_handle_irq(irq);
	}

	/* AT91 specific workaround */
	irq_finish(irq);

#ifdef CONFIG_MTK_SCHED_TRACERS
	if (unlikely(!sched_stopped)) {
	    local_irq_save(flags);
	    if((hardirq_count()>>HARDIRQ_SHIFT) > 1){ //nest state
		g_last_irq = last_irq;
		trace_int_nest(irq, last_irq);
	    }
	    else{
		g_last_irq = last_irq;
		trace_int_switch(mtk_next_task, irq, 0);
	    }
	    local_irq_restore(flags);
	}
	irq_exit();
#else
	irq_exit();
#endif
	set_irq_regs(old_regs);
}

void set_irq_flags(unsigned int irq, unsigned int iflags)
{
	struct irq_desc *desc;
	unsigned long flags;

	if (irq >= NR_IRQS) {
		printk(KERN_ERR "Trying to set irq flags for IRQ%d\n", irq);
		return;
	}

	desc = irq_desc + irq;
	raw_spin_lock_irqsave(&desc->lock, flags);
	desc->status |= IRQ_NOREQUEST | IRQ_NOPROBE | IRQ_NOAUTOEN;
	if (iflags & IRQF_VALID)
		desc->status &= ~IRQ_NOREQUEST;
	if (iflags & IRQF_PROBE)
		desc->status &= ~IRQ_NOPROBE;
	if (!(iflags & IRQF_NOAUTOEN))
		desc->status &= ~IRQ_NOAUTOEN;
	raw_spin_unlock_irqrestore(&desc->lock, flags);
}

void __init init_IRQ(void)
{
	int irq;

	for (irq = 0; irq < NR_IRQS; irq++)
		irq_desc[irq].status |= IRQ_NOREQUEST | IRQ_NOPROBE;

	init_arch_irq();
}

#ifdef CONFIG_HOTPLUG_CPU

static void route_irq(struct irq_desc *desc, unsigned int irq, unsigned int cpu)
{
	pr_debug("IRQ%u: moving from cpu%u to cpu%u\n", irq, desc->node, cpu);

	raw_spin_lock_irq(&desc->lock);
	desc->chip->set_affinity(irq, cpumask_of(cpu));
	raw_spin_unlock_irq(&desc->lock);
}

void migrate_irqs(void)
{
	unsigned int i, cpu = smp_processor_id();

	for (i = 0; i < NR_IRQS; i++) {
		struct irq_desc *desc = irq_desc + i;

		if (desc->node == cpu) {
			unsigned int newcpu = cpumask_any_and(desc->affinity,
							      cpu_online_mask);
			if (newcpu >= nr_cpu_ids) {
				if (printk_ratelimit())
					printk(KERN_INFO "IRQ%u no longer affine to CPU%u\n",
					       i, cpu);

				cpumask_setall(desc->affinity);
				newcpu = cpumask_any_and(desc->affinity,
							 cpu_online_mask);
			}

			route_irq(desc, i, newcpu);
		}
	}
}
#endif /* CONFIG_HOTPLUG_CPU */
