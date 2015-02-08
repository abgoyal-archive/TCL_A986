#ifndef _I386_SEMAPHORE_H
#define _I386_SEMAPHORE_H

#include <linux/linkage.h>

#ifdef __KERNEL__


#include <asm/system.h>
#include <asm/atomic.h>
#include <linux/wait.h>
#include <linux/rwsem.h>

struct semaphore {
	atomic_t count;
	int sleepers;
	wait_queue_head_t wait;
};


#define __SEMAPHORE_INITIALIZER(name, n)				\
{									\
	.count		= ATOMIC_INIT(n),				\
	.sleepers	= 0,						\
	.wait		= __WAIT_QUEUE_HEAD_INITIALIZER((name).wait)	\
}

#define __DECLARE_SEMAPHORE_GENERIC(name,count) \
	struct semaphore name = __SEMAPHORE_INITIALIZER(name,count)

#define DECLARE_MUTEX(name) __DECLARE_SEMAPHORE_GENERIC(name,1)

static inline void sema_init (struct semaphore *sem, int val)
{
	atomic_set(&sem->count, val);
	sem->sleepers = 0;
	init_waitqueue_head(&sem->wait);
}

static inline void init_MUTEX (struct semaphore *sem)
{
	sema_init(sem, 1);
}

static inline void init_MUTEX_LOCKED (struct semaphore *sem)
{
	sema_init(sem, 0);
}

fastcall void __down_failed(void /* special register calling convention */);
fastcall int  __down_failed_interruptible(void  /* params in registers */);
fastcall int  __down_failed_trylock(void  /* params in registers */);
fastcall void __up_wakeup(void /* special register calling convention */);

static inline void down(struct semaphore * sem)
{
	might_sleep();
	__asm__ __volatile__(
		"# atomic down operation\n\t"
		LOCK_PREFIX "decl %0\n\t"     /* --sem->count */
		"jns 2f\n"
		"\tlea %0,%%eax\n\t"
		"call __down_failed\n"
		"2:"
		:"+m" (sem->count)
		:
		:"memory","ax");
}

static inline int down_interruptible(struct semaphore * sem)
{
	int result;

	might_sleep();
	__asm__ __volatile__(
		"# atomic interruptible down operation\n\t"
		"xorl %0,%0\n\t"
		LOCK_PREFIX "decl %1\n\t"     /* --sem->count */
		"jns 2f\n\t"
		"lea %1,%%eax\n\t"
		"call __down_failed_interruptible\n"
		"2:"
		:"=&a" (result), "+m" (sem->count)
		:
		:"memory");
	return result;
}

static inline int down_trylock(struct semaphore * sem)
{
	int result;

	__asm__ __volatile__(
		"# atomic interruptible down operation\n\t"
		"xorl %0,%0\n\t"
		LOCK_PREFIX "decl %1\n\t"     /* --sem->count */
		"jns 2f\n\t"
		"lea %1,%%eax\n\t"
		"call __down_failed_trylock\n\t"
		"2:\n"
		:"=&a" (result), "+m" (sem->count)
		:
		:"memory");
	return result;
}

static inline void up(struct semaphore * sem)
{
	__asm__ __volatile__(
		"# atomic up operation\n\t"
		LOCK_PREFIX "incl %0\n\t"     /* ++sem->count */
		"jg 1f\n\t"
		"lea %0,%%eax\n\t"
		"call __up_wakeup\n"
		"1:"
		:"+m" (sem->count)
		:
		:"memory","ax");
}

#endif
#endif
