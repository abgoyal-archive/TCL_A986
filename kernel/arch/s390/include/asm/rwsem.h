
#ifndef _S390_RWSEM_H
#define _S390_RWSEM_H



#ifndef _LINUX_RWSEM_H
#error "please don't include asm/rwsem.h directly, use linux/rwsem.h instead"
#endif

#ifdef __KERNEL__

#include <linux/list.h>
#include <linux/spinlock.h>

struct rwsem_waiter;

extern struct rw_semaphore *rwsem_down_read_failed(struct rw_semaphore *);
extern struct rw_semaphore *rwsem_down_write_failed(struct rw_semaphore *);
extern struct rw_semaphore *rwsem_wake(struct rw_semaphore *);
extern struct rw_semaphore *rwsem_downgrade_wake(struct rw_semaphore *);
extern struct rw_semaphore *rwsem_downgrade_write(struct rw_semaphore *);

struct rw_semaphore {
	signed long		count;
	spinlock_t		wait_lock;
	struct list_head	wait_list;
#ifdef CONFIG_DEBUG_LOCK_ALLOC
	struct lockdep_map	dep_map;
#endif
};

#ifndef __s390x__
#define RWSEM_UNLOCKED_VALUE	0x00000000
#define RWSEM_ACTIVE_BIAS	0x00000001
#define RWSEM_ACTIVE_MASK	0x0000ffff
#define RWSEM_WAITING_BIAS	(-0x00010000)
#else /* __s390x__ */
#define RWSEM_UNLOCKED_VALUE	0x0000000000000000L
#define RWSEM_ACTIVE_BIAS	0x0000000000000001L
#define RWSEM_ACTIVE_MASK	0x00000000ffffffffL
#define RWSEM_WAITING_BIAS	(-0x0000000100000000L)
#endif /* __s390x__ */
#define RWSEM_ACTIVE_READ_BIAS	RWSEM_ACTIVE_BIAS
#define RWSEM_ACTIVE_WRITE_BIAS	(RWSEM_WAITING_BIAS + RWSEM_ACTIVE_BIAS)


#ifdef CONFIG_DEBUG_LOCK_ALLOC
# define __RWSEM_DEP_MAP_INIT(lockname) , .dep_map = { .name = #lockname }
#else
# define __RWSEM_DEP_MAP_INIT(lockname)
#endif

#define __RWSEM_INITIALIZER(name) \
 { RWSEM_UNLOCKED_VALUE, __SPIN_LOCK_UNLOCKED((name).wait.lock), \
   LIST_HEAD_INIT((name).wait_list) __RWSEM_DEP_MAP_INIT(name) }

#define DECLARE_RWSEM(name) \
	struct rw_semaphore name = __RWSEM_INITIALIZER(name)

static inline void init_rwsem(struct rw_semaphore *sem)
{
	sem->count = RWSEM_UNLOCKED_VALUE;
	spin_lock_init(&sem->wait_lock);
	INIT_LIST_HEAD(&sem->wait_list);
}

extern void __init_rwsem(struct rw_semaphore *sem, const char *name,
			 struct lock_class_key *key);

#define init_rwsem(sem)				\
do {						\
	static struct lock_class_key __key;	\
						\
	__init_rwsem((sem), #sem, &__key);	\
} while (0)


static inline void __down_read(struct rw_semaphore *sem)
{
	signed long old, new;

	asm volatile(
#ifndef __s390x__
		"	l	%0,%2\n"
		"0:	lr	%1,%0\n"
		"	ahi	%1,%4\n"
		"	cs	%0,%1,%2\n"
		"	jl	0b"
#else /* __s390x__ */
		"	lg	%0,%2\n"
		"0:	lgr	%1,%0\n"
		"	aghi	%1,%4\n"
		"	csg	%0,%1,%2\n"
		"	jl	0b"
#endif /* __s390x__ */
		: "=&d" (old), "=&d" (new), "=Q" (sem->count)
		: "Q" (sem->count), "i" (RWSEM_ACTIVE_READ_BIAS)
		: "cc", "memory");
	if (old < 0)
		rwsem_down_read_failed(sem);
}

static inline int __down_read_trylock(struct rw_semaphore *sem)
{
	signed long old, new;

	asm volatile(
#ifndef __s390x__
		"	l	%0,%2\n"
		"0:	ltr	%1,%0\n"
		"	jm	1f\n"
		"	ahi	%1,%4\n"
		"	cs	%0,%1,%2\n"
		"	jl	0b\n"
		"1:"
#else /* __s390x__ */
		"	lg	%0,%2\n"
		"0:	ltgr	%1,%0\n"
		"	jm	1f\n"
		"	aghi	%1,%4\n"
		"	csg	%0,%1,%2\n"
		"	jl	0b\n"
		"1:"
#endif /* __s390x__ */
		: "=&d" (old), "=&d" (new), "=Q" (sem->count)
		: "Q" (sem->count), "i" (RWSEM_ACTIVE_READ_BIAS)
		: "cc", "memory");
	return old >= 0 ? 1 : 0;
}

static inline void __down_write_nested(struct rw_semaphore *sem, int subclass)
{
	signed long old, new, tmp;

	tmp = RWSEM_ACTIVE_WRITE_BIAS;
	asm volatile(
#ifndef __s390x__
		"	l	%0,%2\n"
		"0:	lr	%1,%0\n"
		"	a	%1,%4\n"
		"	cs	%0,%1,%2\n"
		"	jl	0b"
#else /* __s390x__ */
		"	lg	%0,%2\n"
		"0:	lgr	%1,%0\n"
		"	ag	%1,%4\n"
		"	csg	%0,%1,%2\n"
		"	jl	0b"
#endif /* __s390x__ */
		: "=&d" (old), "=&d" (new), "=Q" (sem->count)
		: "Q" (sem->count), "m" (tmp)
		: "cc", "memory");
	if (old != 0)
		rwsem_down_write_failed(sem);
}

static inline void __down_write(struct rw_semaphore *sem)
{
	__down_write_nested(sem, 0);
}

static inline int __down_write_trylock(struct rw_semaphore *sem)
{
	signed long old;

	asm volatile(
#ifndef __s390x__
		"	l	%0,%1\n"
		"0:	ltr	%0,%0\n"
		"	jnz	1f\n"
		"	cs	%0,%3,%1\n"
		"	jl	0b\n"
#else /* __s390x__ */
		"	lg	%0,%1\n"
		"0:	ltgr	%0,%0\n"
		"	jnz	1f\n"
		"	csg	%0,%3,%1\n"
		"	jl	0b\n"
#endif /* __s390x__ */
		"1:"
		: "=&d" (old), "=Q" (sem->count)
		: "Q" (sem->count), "d" (RWSEM_ACTIVE_WRITE_BIAS)
		: "cc", "memory");
	return (old == RWSEM_UNLOCKED_VALUE) ? 1 : 0;
}

static inline void __up_read(struct rw_semaphore *sem)
{
	signed long old, new;

	asm volatile(
#ifndef __s390x__
		"	l	%0,%2\n"
		"0:	lr	%1,%0\n"
		"	ahi	%1,%4\n"
		"	cs	%0,%1,%2\n"
		"	jl	0b"
#else /* __s390x__ */
		"	lg	%0,%2\n"
		"0:	lgr	%1,%0\n"
		"	aghi	%1,%4\n"
		"	csg	%0,%1,%2\n"
		"	jl	0b"
#endif /* __s390x__ */
		: "=&d" (old), "=&d" (new), "=Q" (sem->count)
		: "Q" (sem->count), "i" (-RWSEM_ACTIVE_READ_BIAS)
		: "cc", "memory");
	if (new < 0)
		if ((new & RWSEM_ACTIVE_MASK) == 0)
			rwsem_wake(sem);
}

static inline void __up_write(struct rw_semaphore *sem)
{
	signed long old, new, tmp;

	tmp = -RWSEM_ACTIVE_WRITE_BIAS;
	asm volatile(
#ifndef __s390x__
		"	l	%0,%2\n"
		"0:	lr	%1,%0\n"
		"	a	%1,%4\n"
		"	cs	%0,%1,%2\n"
		"	jl	0b"
#else /* __s390x__ */
		"	lg	%0,%2\n"
		"0:	lgr	%1,%0\n"
		"	ag	%1,%4\n"
		"	csg	%0,%1,%2\n"
		"	jl	0b"
#endif /* __s390x__ */
		: "=&d" (old), "=&d" (new), "=Q" (sem->count)
		: "Q" (sem->count), "m" (tmp)
		: "cc", "memory");
	if (new < 0)
		if ((new & RWSEM_ACTIVE_MASK) == 0)
			rwsem_wake(sem);
}

static inline void __downgrade_write(struct rw_semaphore *sem)
{
	signed long old, new, tmp;

	tmp = -RWSEM_WAITING_BIAS;
	asm volatile(
#ifndef __s390x__
		"	l	%0,%2\n"
		"0:	lr	%1,%0\n"
		"	a	%1,%4\n"
		"	cs	%0,%1,%2\n"
		"	jl	0b"
#else /* __s390x__ */
		"	lg	%0,%2\n"
		"0:	lgr	%1,%0\n"
		"	ag	%1,%4\n"
		"	csg	%0,%1,%2\n"
		"	jl	0b"
#endif /* __s390x__ */
		: "=&d" (old), "=&d" (new), "=Q" (sem->count)
		: "Q" (sem->count), "m" (tmp)
		: "cc", "memory");
	if (new > 1)
		rwsem_downgrade_wake(sem);
}

static inline void rwsem_atomic_add(long delta, struct rw_semaphore *sem)
{
	signed long old, new;

	asm volatile(
#ifndef __s390x__
		"	l	%0,%2\n"
		"0:	lr	%1,%0\n"
		"	ar	%1,%4\n"
		"	cs	%0,%1,%2\n"
		"	jl	0b"
#else /* __s390x__ */
		"	lg	%0,%2\n"
		"0:	lgr	%1,%0\n"
		"	agr	%1,%4\n"
		"	csg	%0,%1,%2\n"
		"	jl	0b"
#endif /* __s390x__ */
		: "=&d" (old), "=&d" (new), "=Q" (sem->count)
		: "Q" (sem->count), "d" (delta)
		: "cc", "memory");
}

static inline long rwsem_atomic_update(long delta, struct rw_semaphore *sem)
{
	signed long old, new;

	asm volatile(
#ifndef __s390x__
		"	l	%0,%2\n"
		"0:	lr	%1,%0\n"
		"	ar	%1,%4\n"
		"	cs	%0,%1,%2\n"
		"	jl	0b"
#else /* __s390x__ */
		"	lg	%0,%2\n"
		"0:	lgr	%1,%0\n"
		"	agr	%1,%4\n"
		"	csg	%0,%1,%2\n"
		"	jl	0b"
#endif /* __s390x__ */
		: "=&d" (old), "=&d" (new), "=Q" (sem->count)
		: "Q" (sem->count), "d" (delta)
		: "cc", "memory");
	return new;
}

static inline int rwsem_is_locked(struct rw_semaphore *sem)
{
	return (sem->count != 0);
}

#endif /* __KERNEL__ */
#endif /* _S390_RWSEM_H */
