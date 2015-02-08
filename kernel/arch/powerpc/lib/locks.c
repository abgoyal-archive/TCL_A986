

#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/module.h>
#include <linux/stringify.h>
#include <linux/smp.h>

/* waiting for a spinlock... */
#if defined(CONFIG_PPC_SPLPAR) || defined(CONFIG_PPC_ISERIES)
#include <asm/hvcall.h>
#include <asm/iseries/hv_call.h>
#include <asm/smp.h>
#include <asm/firmware.h>

void __spin_yield(arch_spinlock_t *lock)
{
	unsigned int lock_value, holder_cpu, yield_count;

	lock_value = lock->slock;
	if (lock_value == 0)
		return;
	holder_cpu = lock_value & 0xffff;
	BUG_ON(holder_cpu >= NR_CPUS);
	yield_count = lppaca[holder_cpu].yield_count;
	if ((yield_count & 1) == 0)
		return;		/* virtual cpu is currently running */
	rmb();
	if (lock->slock != lock_value)
		return;		/* something has changed */
	if (firmware_has_feature(FW_FEATURE_ISERIES))
		HvCall2(HvCallBaseYieldProcessor, HvCall_YieldToProc,
			((u64)holder_cpu << 32) | yield_count);
#ifdef CONFIG_PPC_SPLPAR
	else
		plpar_hcall_norets(H_CONFER,
			get_hard_smp_processor_id(holder_cpu), yield_count);
#endif
}

void __rw_yield(arch_rwlock_t *rw)
{
	int lock_value;
	unsigned int holder_cpu, yield_count;

	lock_value = rw->lock;
	if (lock_value >= 0)
		return;		/* no write lock at present */
	holder_cpu = lock_value & 0xffff;
	BUG_ON(holder_cpu >= NR_CPUS);
	yield_count = lppaca[holder_cpu].yield_count;
	if ((yield_count & 1) == 0)
		return;		/* virtual cpu is currently running */
	rmb();
	if (rw->lock != lock_value)
		return;		/* something has changed */
	if (firmware_has_feature(FW_FEATURE_ISERIES))
		HvCall2(HvCallBaseYieldProcessor, HvCall_YieldToProc,
			((u64)holder_cpu << 32) | yield_count);
#ifdef CONFIG_PPC_SPLPAR
	else
		plpar_hcall_norets(H_CONFER,
			get_hard_smp_processor_id(holder_cpu), yield_count);
#endif
}
#endif

void arch_spin_unlock_wait(arch_spinlock_t *lock)
{
	while (lock->slock) {
		HMT_low();
		if (SHARED_PROCESSOR)
			__spin_yield(lock);
	}
	HMT_medium();
}

EXPORT_SYMBOL(arch_spin_unlock_wait);
