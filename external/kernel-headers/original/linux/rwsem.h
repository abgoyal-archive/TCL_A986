
#ifndef _LINUX_RWSEM_H
#define _LINUX_RWSEM_H

#include <linux/linkage.h>

#ifdef __KERNEL__

#include <linux/types.h>
#include <linux/kernel.h>
#include <asm/system.h>
#include <asm/atomic.h>

struct rw_semaphore;

#ifdef CONFIG_RWSEM_GENERIC_SPINLOCK
#include <linux/rwsem-spinlock.h> /* use a generic implementation */
#else
#include <asm/rwsem.h> /* use an arch-specific implementation */
#endif

extern void down_read(struct rw_semaphore *sem);

extern int down_read_trylock(struct rw_semaphore *sem);

extern void down_write(struct rw_semaphore *sem);

extern int down_write_trylock(struct rw_semaphore *sem);

extern void up_read(struct rw_semaphore *sem);

extern void up_write(struct rw_semaphore *sem);

extern void downgrade_write(struct rw_semaphore *sem);

#ifdef CONFIG_DEBUG_LOCK_ALLOC
extern void down_read_nested(struct rw_semaphore *sem, int subclass);
extern void down_write_nested(struct rw_semaphore *sem, int subclass);
extern void down_read_non_owner(struct rw_semaphore *sem);
extern void up_read_non_owner(struct rw_semaphore *sem);
#else
# define down_read_nested(sem, subclass)		down_read(sem)
# define down_write_nested(sem, subclass)	down_write(sem)
# define down_read_non_owner(sem)		down_read(sem)
# define up_read_non_owner(sem)			up_read(sem)
#endif

#endif /* __KERNEL__ */
#endif /* _LINUX_RWSEM_H */
