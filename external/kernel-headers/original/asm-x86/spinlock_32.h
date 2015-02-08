#ifndef __ASM_SPINLOCK_H
#define __ASM_SPINLOCK_H

#include <asm/atomic.h>
#include <asm/rwlock.h>
#include <asm/page.h>
#include <asm/processor.h>
#include <linux/compiler.h>

#ifdef CONFIG_PARAVIRT
#include <asm/paravirt.h>
#else
#define CLI_STRING	"cli"
#define STI_STRING	"sti"
#define CLI_STI_CLOBBERS
#define CLI_STI_INPUT_ARGS
#endif /* CONFIG_PARAVIRT */


static inline int __raw_spin_is_locked(raw_spinlock_t *x)
{
	return *(volatile signed char *)(&(x)->slock) <= 0;
}

static inline void __raw_spin_lock(raw_spinlock_t *lock)
{
	asm volatile("\n1:\t"
		     LOCK_PREFIX " ; decb %0\n\t"
		     "jns 3f\n"
		     "2:\t"
		     "rep;nop\n\t"
		     "cmpb $0,%0\n\t"
		     "jle 2b\n\t"
		     "jmp 1b\n"
		     "3:\n\t"
		     : "+m" (lock->slock) : : "memory");
}

#ifndef CONFIG_PROVE_LOCKING
static inline void __raw_spin_lock_flags(raw_spinlock_t *lock, unsigned long flags)
{
	asm volatile(
		"\n1:\t"
		LOCK_PREFIX " ; decb %[slock]\n\t"
		"jns 5f\n"
		"2:\t"
		"testl $0x200, %[flags]\n\t"
		"jz 4f\n\t"
		STI_STRING "\n"
		"3:\t"
		"rep;nop\n\t"
		"cmpb $0, %[slock]\n\t"
		"jle 3b\n\t"
		CLI_STRING "\n\t"
		"jmp 1b\n"
		"4:\t"
		"rep;nop\n\t"
		"cmpb $0, %[slock]\n\t"
		"jg 1b\n\t"
		"jmp 4b\n"
		"5:\n\t"
		: [slock] "+m" (lock->slock)
		: [flags] "r" (flags)
	 	  CLI_STI_INPUT_ARGS
		: "memory" CLI_STI_CLOBBERS);
}
#endif

static inline int __raw_spin_trylock(raw_spinlock_t *lock)
{
	char oldval;
	asm volatile(
		"xchgb %b0,%1"
		:"=q" (oldval), "+m" (lock->slock)
		:"0" (0) : "memory");
	return oldval > 0;
}


#if !defined(CONFIG_X86_OOSTORE) && !defined(CONFIG_X86_PPRO_FENCE)

static inline void __raw_spin_unlock(raw_spinlock_t *lock)
{
	asm volatile("movb $1,%0" : "+m" (lock->slock) :: "memory");
}

#else

static inline void __raw_spin_unlock(raw_spinlock_t *lock)
{
	char oldval = 1;

	asm volatile("xchgb %b0, %1"
		     : "=q" (oldval), "+m" (lock->slock)
		     : "0" (oldval) : "memory");
}

#endif

static inline void __raw_spin_unlock_wait(raw_spinlock_t *lock)
{
	while (__raw_spin_is_locked(lock))
		cpu_relax();
}


static inline int __raw_read_can_lock(raw_rwlock_t *x)
{
	return (int)(x)->lock > 0;
}

static inline int __raw_write_can_lock(raw_rwlock_t *x)
{
	return (x)->lock == RW_LOCK_BIAS;
}

static inline void __raw_read_lock(raw_rwlock_t *rw)
{
	asm volatile(LOCK_PREFIX " subl $1,(%0)\n\t"
		     "jns 1f\n"
		     "call __read_lock_failed\n\t"
		     "1:\n"
		     ::"a" (rw) : "memory");
}

static inline void __raw_write_lock(raw_rwlock_t *rw)
{
	asm volatile(LOCK_PREFIX " subl $" RW_LOCK_BIAS_STR ",(%0)\n\t"
		     "jz 1f\n"
		     "call __write_lock_failed\n\t"
		     "1:\n"
		     ::"a" (rw) : "memory");
}

static inline int __raw_read_trylock(raw_rwlock_t *lock)
{
	atomic_t *count = (atomic_t *)lock;
	atomic_dec(count);
	if (atomic_read(count) >= 0)
		return 1;
	atomic_inc(count);
	return 0;
}

static inline int __raw_write_trylock(raw_rwlock_t *lock)
{
	atomic_t *count = (atomic_t *)lock;
	if (atomic_sub_and_test(RW_LOCK_BIAS, count))
		return 1;
	atomic_add(RW_LOCK_BIAS, count);
	return 0;
}

static inline void __raw_read_unlock(raw_rwlock_t *rw)
{
	asm volatile(LOCK_PREFIX "incl %0" :"+m" (rw->lock) : : "memory");
}

static inline void __raw_write_unlock(raw_rwlock_t *rw)
{
	asm volatile(LOCK_PREFIX "addl $" RW_LOCK_BIAS_STR ", %0"
				 : "+m" (rw->lock) : : "memory");
}

#define _raw_spin_relax(lock)	cpu_relax()
#define _raw_read_relax(lock)	cpu_relax()
#define _raw_write_relax(lock)	cpu_relax()

#endif /* __ASM_SPINLOCK_H */
