

#include <linux/init.h>

#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/kernel_stat.h>
#include <linux/mc146818rtc.h>
#include <linux/cache.h>
#include <linux/interrupt.h>
#include <linux/cpu.h>
#include <linux/gfp.h>

#include <asm/mtrr.h>
#include <asm/tlbflush.h>
#include <asm/mmu_context.h>
#include <asm/proto.h>
#include <asm/apic.h>

static void native_smp_send_reschedule(int cpu)
{
	if (unlikely(cpu_is_offline(cpu))) {
		WARN_ON(1);
		return;
	}
	apic->send_IPI_mask(cpumask_of(cpu), RESCHEDULE_VECTOR);
}

void native_send_call_func_single_ipi(int cpu)
{
	apic->send_IPI_mask(cpumask_of(cpu), CALL_FUNCTION_SINGLE_VECTOR);
}

void native_send_call_func_ipi(const struct cpumask *mask)
{
	cpumask_var_t allbutself;

	if (!alloc_cpumask_var(&allbutself, GFP_ATOMIC)) {
		apic->send_IPI_mask(mask, CALL_FUNCTION_VECTOR);
		return;
	}

	cpumask_copy(allbutself, cpu_online_mask);
	cpumask_clear_cpu(smp_processor_id(), allbutself);

	if (cpumask_equal(mask, allbutself) &&
	    cpumask_equal(cpu_online_mask, cpu_callout_mask))
		apic->send_IPI_allbutself(CALL_FUNCTION_VECTOR);
	else
		apic->send_IPI_mask(mask, CALL_FUNCTION_VECTOR);

	free_cpumask_var(allbutself);
}


asmlinkage void smp_reboot_interrupt(void)
{
	ack_APIC_irq();
	irq_enter();
	stop_this_cpu(NULL);
	irq_exit();
}

static void native_smp_send_stop(void)
{
	unsigned long flags;
	unsigned long wait;

	if (reboot_force)
		return;

	/*
	 * Use an own vector here because smp_call_function
	 * does lots of things not suitable in a panic situation.
	 * On most systems we could also use an NMI here,
	 * but there are a few systems around where NMI
	 * is problematic so stay with an non NMI for now
	 * (this implies we cannot stop CPUs spinning with irq off
	 * currently)
	 */
	if (num_online_cpus() > 1) {
		apic->send_IPI_allbutself(REBOOT_VECTOR);

		/* Don't wait longer than a second */
		wait = USEC_PER_SEC;
		while (num_online_cpus() > 1 && wait--)
			udelay(1);
	}

	local_irq_save(flags);
	disable_local_APIC();
	local_irq_restore(flags);
}

void smp_reschedule_interrupt(struct pt_regs *regs)
{
	ack_APIC_irq();
	inc_irq_stat(irq_resched_count);
	/*
	 * KVM uses this interrupt to force a cpu out of guest mode
	 */
}

void smp_call_function_interrupt(struct pt_regs *regs)
{
	ack_APIC_irq();
	irq_enter();
	generic_smp_call_function_interrupt();
	inc_irq_stat(irq_call_count);
	irq_exit();
}

void smp_call_function_single_interrupt(struct pt_regs *regs)
{
	ack_APIC_irq();
	irq_enter();
	generic_smp_call_function_single_interrupt();
	inc_irq_stat(irq_call_count);
	irq_exit();
}

struct smp_ops smp_ops = {
	.smp_prepare_boot_cpu	= native_smp_prepare_boot_cpu,
	.smp_prepare_cpus	= native_smp_prepare_cpus,
	.smp_cpus_done		= native_smp_cpus_done,

	.smp_send_stop		= native_smp_send_stop,
	.smp_send_reschedule	= native_smp_send_reschedule,

	.cpu_up			= native_cpu_up,
	.cpu_die		= native_cpu_die,
	.cpu_disable		= native_cpu_disable,
	.play_dead		= native_play_dead,

	.send_call_func_ipi	= native_send_call_func_ipi,
	.send_call_func_single_ipi = native_send_call_func_single_ipi,
};
EXPORT_SYMBOL_GPL(smp_ops);
