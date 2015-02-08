
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel_stat.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/mm.h>
#include <linux/random.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/kallsyms.h>
#include <linux/kgdb.h>
#include <linux/ftrace.h>

#include <asm/atomic.h>
#include <asm/system.h>
#include <asm/uaccess.h>

#ifdef CONFIG_KGDB
int kgdb_early_setup;
#endif

static unsigned long irq_map[NR_IRQS / BITS_PER_LONG];

int allocate_irqno(void)
{
	int irq;

again:
	irq = find_first_zero_bit(irq_map, NR_IRQS);

	if (irq >= NR_IRQS)
		return -ENOSPC;

	if (test_and_set_bit(irq, irq_map))
		goto again;

	return irq;
}

void __init alloc_legacy_irqno(void)
{
	int i;

	for (i = 0; i <= 16; i++)
		BUG_ON(test_and_set_bit(i, irq_map));
}

void free_irqno(unsigned int irq)
{
	smp_mb__before_clear_bit();
	clear_bit(irq, irq_map);
	smp_mb__after_clear_bit();
}

void ack_bad_irq(unsigned int irq)
{
	smtc_im_ack_irq(irq);
	printk("unexpected IRQ # %d\n", irq);
}

atomic_t irq_err_count;


int show_interrupts(struct seq_file *p, void *v)
{
	int i = *(loff_t *) v, j;
	struct irqaction * action;
	unsigned long flags;

	if (i == 0) {
		seq_printf(p, "           ");
		for_each_online_cpu(j)
			seq_printf(p, "CPU%d       ", j);
		seq_putc(p, '\n');
	}

	if (i < NR_IRQS) {
		raw_spin_lock_irqsave(&irq_desc[i].lock, flags);
		action = irq_desc[i].action;
		if (!action)
			goto skip;
		seq_printf(p, "%3d: ", i);
#ifndef CONFIG_SMP
		seq_printf(p, "%10u ", kstat_irqs(i));
#else
		for_each_online_cpu(j)
			seq_printf(p, "%10u ", kstat_irqs_cpu(i, j));
#endif
		seq_printf(p, " %14s", irq_desc[i].chip->name);
		seq_printf(p, "  %s", action->name);

		for (action=action->next; action; action = action->next)
			seq_printf(p, ", %s", action->name);

		seq_putc(p, '\n');
skip:
		raw_spin_unlock_irqrestore(&irq_desc[i].lock, flags);
	} else if (i == NR_IRQS) {
		seq_putc(p, '\n');
		seq_printf(p, "ERR: %10u\n", atomic_read(&irq_err_count));
	}
	return 0;
}

asmlinkage void spurious_interrupt(void)
{
	atomic_inc(&irq_err_count);
}

void __init init_IRQ(void)
{
	int i;

#ifdef CONFIG_KGDB
	if (kgdb_early_setup)
		return;
#endif

	for (i = 0; i < NR_IRQS; i++)
		set_irq_noprobe(i);

	arch_init_irq();

#ifdef CONFIG_KGDB
	if (!kgdb_early_setup)
		kgdb_early_setup = 1;
#endif
}

void __irq_entry do_IRQ(unsigned int irq)
{
	irq_enter();
	__DO_IRQ_SMTC_HOOK(irq);
	generic_handle_irq(irq);
	irq_exit();
}

#ifdef CONFIG_MIPS_MT_SMTC_IRQAFF

void __irq_entry do_IRQ_no_affinity(unsigned int irq)
{
	irq_enter();
	__NO_AFFINITY_IRQ_SMTC_HOOK(irq);
	generic_handle_irq(irq);
	irq_exit();
}

#endif /* CONFIG_MIPS_MT_SMTC_IRQAFF */
