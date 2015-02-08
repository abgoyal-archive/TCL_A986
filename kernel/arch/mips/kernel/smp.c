
#include <linux/cache.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/smp.h>
#include <linux/spinlock.h>
#include <linux/threads.h>
#include <linux/module.h>
#include <linux/time.h>
#include <linux/timex.h>
#include <linux/sched.h>
#include <linux/cpumask.h>
#include <linux/cpu.h>
#include <linux/err.h>
#include <linux/ftrace.h>

#include <asm/atomic.h>
#include <asm/cpu.h>
#include <asm/processor.h>
#include <asm/r4k-timer.h>
#include <asm/system.h>
#include <asm/mmu_context.h>
#include <asm/time.h>

#ifdef CONFIG_MIPS_MT_SMTC
#include <asm/mipsmtregs.h>
#endif /* CONFIG_MIPS_MT_SMTC */

volatile cpumask_t cpu_callin_map;	/* Bitmask of started secondaries */
int __cpu_number_map[NR_CPUS];		/* Map physical to logical */
int __cpu_logical_map[NR_CPUS];		/* Map logical to physical */

/* Number of TCs (or siblings in Intel speak) per CPU core */
int smp_num_siblings = 1;
EXPORT_SYMBOL(smp_num_siblings);

/* representing the TCs (or siblings in Intel speak) of each logical CPU */
cpumask_t cpu_sibling_map[NR_CPUS] __read_mostly;
EXPORT_SYMBOL(cpu_sibling_map);

/* representing cpus for which sibling maps can be computed */
static cpumask_t cpu_sibling_setup_map;

static inline void set_cpu_sibling_map(int cpu)
{
	int i;

	cpu_set(cpu, cpu_sibling_setup_map);

	if (smp_num_siblings > 1) {
		for_each_cpu_mask(i, cpu_sibling_setup_map) {
			if (cpu_data[cpu].core == cpu_data[i].core) {
				cpu_set(i, cpu_sibling_map[cpu]);
				cpu_set(cpu, cpu_sibling_map[i]);
			}
		}
	} else
		cpu_set(cpu, cpu_sibling_map[cpu]);
}

struct plat_smp_ops *mp_ops;

__cpuinit void register_smp_ops(struct plat_smp_ops *ops)
{
	if (mp_ops)
		printk(KERN_WARNING "Overriding previously set SMP ops\n");

	mp_ops = ops;
}

asmlinkage __cpuinit void start_secondary(void)
{
	unsigned int cpu;

#ifdef CONFIG_MIPS_MT_SMTC
	/* Only do cpu_probe for first TC of CPU */
	if ((read_c0_tcbind() & TCBIND_CURTC) == 0)
#endif /* CONFIG_MIPS_MT_SMTC */
	cpu_probe();
	cpu_report();
	per_cpu_trap_init();
	mips_clockevent_init();
	mp_ops->init_secondary();

	/*
	 * XXX parity protection should be folded in here when it's converted
	 * to an option instead of something based on .cputype
	 */

	calibrate_delay();
	preempt_disable();
	cpu = smp_processor_id();
	cpu_data[cpu].udelay_val = loops_per_jiffy;

	notify_cpu_starting(cpu);

	mp_ops->smp_finish();
	set_cpu_sibling_map(cpu);

	cpu_set(cpu, cpu_callin_map);

	synchronise_count_slave();

	cpu_idle();
}

void __irq_entry smp_call_function_interrupt(void)
{
	irq_enter();
	generic_smp_call_function_single_interrupt();
	generic_smp_call_function_interrupt();
	irq_exit();
}

static void stop_this_cpu(void *dummy)
{
	/*
	 * Remove this CPU:
	 */
	cpu_clear(smp_processor_id(), cpu_online_map);
	for (;;) {
		if (cpu_wait)
			(*cpu_wait)();		/* Wait if available. */
	}
}

void smp_send_stop(void)
{
	smp_call_function(stop_this_cpu, NULL, 0);
}

void __init smp_cpus_done(unsigned int max_cpus)
{
	mp_ops->cpus_done();
	synchronise_count_master();
#ifdef CONFIG_HAVE_UNSYNCHRONIZED_TSC
	test_tsc_synchronization();
#endif
}

/* called from main before smp_init() */
void __init smp_prepare_cpus(unsigned int max_cpus)
{
	init_new_context(current, &init_mm);
	current_thread_info()->cpu = 0;
	mp_ops->prepare_cpus(max_cpus);
	set_cpu_sibling_map(0);
#ifndef CONFIG_HOTPLUG_CPU
	init_cpu_present(&cpu_possible_map);
#endif
}

/* preload SMP state for boot cpu */
void __devinit smp_prepare_boot_cpu(void)
{
	set_cpu_possible(0, true);
	set_cpu_online(0, true);
	cpu_set(0, cpu_callin_map);
}

static struct task_struct *cpu_idle_thread[NR_CPUS];

int __cpuinit __cpu_up(unsigned int cpu)
{
	struct task_struct *idle;

	/*
	 * Processor goes to start_secondary(), sets online flag
	 * The following code is purely to make sure
	 * Linux can schedule processes on this slave.
	 */
	if (!cpu_idle_thread[cpu]) {
		idle = fork_idle(cpu);
		cpu_idle_thread[cpu] = idle;

		if (IS_ERR(idle))
			panic(KERN_ERR "Fork failed for CPU %d", cpu);
	} else {
		idle = cpu_idle_thread[cpu];
		init_idle(idle, cpu);
	}

	mp_ops->boot_secondary(cpu, idle);

	/*
	 * Trust is futile.  We should really have timeouts ...
	 */
	while (!cpu_isset(cpu, cpu_callin_map))
		udelay(100);

	cpu_set(cpu, cpu_online_map);

	return 0;
}

/* Not really SMP stuff ... */
int setup_profiling_timer(unsigned int multiplier)
{
	return 0;
}

static void flush_tlb_all_ipi(void *info)
{
	local_flush_tlb_all();
}

void flush_tlb_all(void)
{
	on_each_cpu(flush_tlb_all_ipi, NULL, 1);
}

static void flush_tlb_mm_ipi(void *mm)
{
	local_flush_tlb_mm((struct mm_struct *)mm);
}

static inline void smp_on_other_tlbs(void (*func) (void *info), void *info)
{
#ifndef CONFIG_MIPS_MT_SMTC
	smp_call_function(func, info, 1);
#endif
}

static inline void smp_on_each_tlb(void (*func) (void *info), void *info)
{
	preempt_disable();

	smp_on_other_tlbs(func, info);
	func(info);

	preempt_enable();
}


void flush_tlb_mm(struct mm_struct *mm)
{
	preempt_disable();

	if ((atomic_read(&mm->mm_users) != 1) || (current->mm != mm)) {
		smp_on_other_tlbs(flush_tlb_mm_ipi, mm);
	} else {
		cpumask_t mask = cpu_online_map;
		unsigned int cpu;

		cpu_clear(smp_processor_id(), mask);
		for_each_cpu_mask(cpu, mask)
			if (cpu_context(cpu, mm))
				cpu_context(cpu, mm) = 0;
	}
	local_flush_tlb_mm(mm);

	preempt_enable();
}

struct flush_tlb_data {
	struct vm_area_struct *vma;
	unsigned long addr1;
	unsigned long addr2;
};

static void flush_tlb_range_ipi(void *info)
{
	struct flush_tlb_data *fd = info;

	local_flush_tlb_range(fd->vma, fd->addr1, fd->addr2);
}

void flush_tlb_range(struct vm_area_struct *vma, unsigned long start, unsigned long end)
{
	struct mm_struct *mm = vma->vm_mm;

	preempt_disable();
	if ((atomic_read(&mm->mm_users) != 1) || (current->mm != mm)) {
		struct flush_tlb_data fd = {
			.vma = vma,
			.addr1 = start,
			.addr2 = end,
		};

		smp_on_other_tlbs(flush_tlb_range_ipi, &fd);
	} else {
		cpumask_t mask = cpu_online_map;
		unsigned int cpu;

		cpu_clear(smp_processor_id(), mask);
		for_each_cpu_mask(cpu, mask)
			if (cpu_context(cpu, mm))
				cpu_context(cpu, mm) = 0;
	}
	local_flush_tlb_range(vma, start, end);
	preempt_enable();
}

static void flush_tlb_kernel_range_ipi(void *info)
{
	struct flush_tlb_data *fd = info;

	local_flush_tlb_kernel_range(fd->addr1, fd->addr2);
}

void flush_tlb_kernel_range(unsigned long start, unsigned long end)
{
	struct flush_tlb_data fd = {
		.addr1 = start,
		.addr2 = end,
	};

	on_each_cpu(flush_tlb_kernel_range_ipi, &fd, 1);
}

static void flush_tlb_page_ipi(void *info)
{
	struct flush_tlb_data *fd = info;

	local_flush_tlb_page(fd->vma, fd->addr1);
}

void flush_tlb_page(struct vm_area_struct *vma, unsigned long page)
{
	preempt_disable();
	if ((atomic_read(&vma->vm_mm->mm_users) != 1) || (current->mm != vma->vm_mm)) {
		struct flush_tlb_data fd = {
			.vma = vma,
			.addr1 = page,
		};

		smp_on_other_tlbs(flush_tlb_page_ipi, &fd);
	} else {
		cpumask_t mask = cpu_online_map;
		unsigned int cpu;

		cpu_clear(smp_processor_id(), mask);
		for_each_cpu_mask(cpu, mask)
			if (cpu_context(cpu, vma->vm_mm))
				cpu_context(cpu, vma->vm_mm) = 0;
	}
	local_flush_tlb_page(vma, page);
	preempt_enable();
}

static void flush_tlb_one_ipi(void *info)
{
	unsigned long vaddr = (unsigned long) info;

	local_flush_tlb_one(vaddr);
}

void flush_tlb_one(unsigned long vaddr)
{
	smp_on_each_tlb(flush_tlb_one_ipi, (void *) vaddr);
}

EXPORT_SYMBOL(flush_tlb_page);
EXPORT_SYMBOL(flush_tlb_one);
