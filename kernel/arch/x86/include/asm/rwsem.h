

#ifndef _ASM_X86_RWSEM_H
#define _ASM_X86_RWSEM_H

#ifndef _LINUX_RWSEM_H
#error "please don't include asm/rwsem.h directly, use linux/rwsem.h instead"
#endif

#ifdef __KERNEL__

#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/lockdep.h>
#include <asm/asm.h>

struct rwsem_waiter;

extern asmregparm struct rw_semaphore *
 rwsem_down_read_failed(struct rw_semaphore *sem);
extern asmregparm struct rw_semaphore *
 rwsem_down_write_failed(struct rw_semaphore *sem);
extern asmregparm struct rw_semaphore *
 rwsem_wake(struct rw_semaphore *);
extern asmregparm struct rw_semaphore *
 rwsem_downgrade_wake(struct rw_semaphore *sem);


#ifdef CONFIG_X86_64
# define RWSEM_ACTIVE_MASK		0xffffffffL
#else
# define RWSEM_ACTIVE_MASK		0x0000ffffL
#endif

#define RWSEM_UNLOCKED_VALUE		0x00000000L
#define RWSEM_ACTIVE_BIAS		0x00000001L
#define RWSEM_WAITING_BIAS		(-RWSEM_ACTIVE_MASK-1)
#define RWSEM_ACTIVE_READ_BIAS		RWSEM_ACTIVE_BIAS
#define RWSEM_ACTIVE_WRITE_BIAS		(RWSEM_WAITING_BIAS + RWSEM_ACTIVE_BIAS)

typedef signed long rwsem_count_t;

struct rw_semaphore {
	rwsem_count_t		count;
	spinlock_t		wait_lock;
	struct list_head	wait_list;
#ifdef CONFIG_DEBUG_LOCK_ALLOC
	struct lockdep_map dep_map;
#endif
};

#ifdef CONFIG_DEBUG_LOCK_ALLOC
# define __RWSEM_DEP_MAP_INIT(lockname) , .dep_map = { .name = #lockname }
#else
# define __RWSEM_DEP_MAP_INIT(lockname)
#endif


#define __RWSEM_INITIALIZER(name)				\
{								\
	RWSEM_UNLOCKED_VALUE, __SPIN_LOCK_UNLOCKED((name).wait_lock), \
	LIST_HEAD_INIT((name).wait_list) __RWSEM_DEP_MAP_INIT(name) \
}

#define DECLARE_RWSEM(name)					\
	struct rw_semaphore name = __RWSEM_INITIALIZER(name)

extern void __init_rwsem(struct rw_semaphore *sem, const char *name,
			 struct lock_class_key *key);

#define init_rwsem(sem)						\
do {								\
	static struct lock_class_key __key;			\
								\
	__init_rwsem((sem), #sem, &__key);			\
} while (0)

static inline void __down_read(struct rw_semaphore *sem)
{
	asm volatile("# beginning down_read\n\t"
		     LOCK_PREFIX _ASM_INC "(%1)\n\t"
		     /* adds 0x00000001, returns the old value */
		     "  jns        1f\n"
		     "  call call_rwsem_down_read_failed\n"
		     "1:\n\t"
		     "# ending down_read\n\t"
		     : "+m" (sem->count)
		     : "a" (sem)
		     : "memory", "cc");
}

static inline int __down_read_trylock(struct rw_semaphore *sem)
{
	rwsem_count_t result, tmp;
	asm volatile("# beginning __down_read_trylock\n\t"
		     "  mov          %0,%1\n\t"
		     "1:\n\t"
		     "  mov          %1,%2\n\t"
		     "  add          %3,%2\n\t"
		     "  jle	     2f\n\t"
		     LOCK_PREFIX "  cmpxchg  %2,%0\n\t"
		     "  jnz	     1b\n\t"
		     "2:\n\t"
		     "# ending __down_read_trylock\n\t"
		     : "+m" (sem->count), "=&a" (result), "=&r" (tmp)
		     : "i" (RWSEM_ACTIVE_READ_BIAS)
		     : "memory", "cc");
	return result >= 0 ? 1 : 0;
}

static inline void __down_write_nested(struct rw_semaphore *sem, int subclass)
{
	rwsem_count_t tmp;

	tmp = RWSEM_ACTIVE_WRITE_BIAS;
	asm volatile("# beginning down_write\n\t"
		     LOCK_PREFIX "  xadd      %1,(%2)\n\t"
		     /* subtract 0x0000ffff, returns the old value */
		     "  test      %1,%1\n\t"
		     /* was the count 0 before? */
		     "  jz        1f\n"
		     "  call call_rwsem_down_write_failed\n"
		     "1:\n"
		     "# ending down_write"
		     : "+m" (sem->count), "=d" (tmp)
		     : "a" (sem), "1" (tmp)
		     : "memory", "cc");
}

static inline void __down_write(struct rw_semaphore *sem)
{
	__down_write_nested(sem, 0);
}

static inline int __down_write_trylock(struct rw_semaphore *sem)
{
	rwsem_count_t ret = cmpxchg(&sem->count,
				    RWSEM_UNLOCKED_VALUE,
				    RWSEM_ACTIVE_WRITE_BIAS);
	if (ret == RWSEM_UNLOCKED_VALUE)
		return 1;
	return 0;
}

static inline void __up_read(struct rw_semaphore *sem)
{
	rwsem_count_t tmp = -RWSEM_ACTIVE_READ_BIAS;
	asm volatile("# beginning __up_read\n\t"
		     LOCK_PREFIX "  xadd      %1,(%2)\n\t"
		     /* subtracts 1, returns the old value */
		     "  jns        1f\n\t"
		     "  call call_rwsem_wake\n"
		     "1:\n"
		     "# ending __up_read\n"
		     : "+m" (sem->count), "=d" (tmp)
		     : "a" (sem), "1" (tmp)
		     : "memory", "cc");
}

static inline void __up_write(struct rw_semaphore *sem)
{
	rwsem_count_t tmp;
	asm volatile("# beginning __up_write\n\t"
		     LOCK_PREFIX "  xadd      %1,(%2)\n\t"
		     /* tries to transition
			0xffff0001 -> 0x00000000 */
		     "  jz       1f\n"
		     "  call call_rwsem_wake\n"
		     "1:\n\t"
		     "# ending __up_write\n"
		     : "+m" (sem->count), "=d" (tmp)
		     : "a" (sem), "1" (-RWSEM_ACTIVE_WRITE_BIAS)
		     : "memory", "cc");
}

static inline void __downgrade_write(struct rw_semaphore *sem)
{
	asm volatile("# beginning __downgrade_write\n\t"
		     LOCK_PREFIX _ASM_ADD "%2,(%1)\n\t"
		     /*
		      * transitions 0xZZZZ0001 -> 0xYYYY0001 (i386)
		      *     0xZZZZZZZZ00000001 -> 0xYYYYYYYY00000001 (x86_64)
		      */
		     "  jns       1f\n\t"
		     "  call call_rwsem_downgrade_wake\n"
		     "1:\n\t"
		     "# ending __downgrade_write\n"
		     : "+m" (sem->count)
		     : "a" (sem), "er" (-RWSEM_WAITING_BIAS)
		     : "memory", "cc");
}

static inline void rwsem_atomic_add(rwsem_count_t delta,
				    struct rw_semaphore *sem)
{
	asm volatile(LOCK_PREFIX _ASM_ADD "%1,%0"
		     : "+m" (sem->count)
		     : "er" (delta));
}

static inline rwsem_count_t rwsem_atomic_update(rwsem_count_t delta,
						struct rw_semaphore *sem)
{
	rwsem_count_t tmp = delta;

	asm volatile(LOCK_PREFIX "xadd %0,%1"
		     : "+r" (tmp), "+m" (sem->count)
		     : : "memory");

	return tmp + delta;
}

static inline int rwsem_is_locked(struct rw_semaphore *sem)
{
	return (sem->count != 0);
}

#endif /* __KERNEL__ */
#endif /* _ASM_X86_RWSEM_H */
