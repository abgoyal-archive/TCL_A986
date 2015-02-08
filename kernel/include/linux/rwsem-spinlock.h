

#ifndef _LINUX_RWSEM_SPINLOCK_H
#define _LINUX_RWSEM_SPINLOCK_H

#ifndef _LINUX_RWSEM_H
#error "please don't include linux/rwsem-spinlock.h directly, use linux/rwsem.h instead"
#endif

#include <linux/spinlock.h>
#include <linux/list.h>

#ifdef __KERNEL__

#include <linux/types.h>

struct rwsem_waiter;

struct rw_semaphore {
	__s32			activity;
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

#define __RWSEM_INITIALIZER(name) \
{ 0, __SPIN_LOCK_UNLOCKED(name.wait_lock), LIST_HEAD_INIT((name).wait_list) \
  __RWSEM_DEP_MAP_INIT(name) }

#define DECLARE_RWSEM(name) \
	struct rw_semaphore name = __RWSEM_INITIALIZER(name)

extern void __init_rwsem(struct rw_semaphore *sem, const char *name,
			 struct lock_class_key *key);

#define init_rwsem(sem)						\
do {								\
	static struct lock_class_key __key;			\
								\
	__init_rwsem((sem), #sem, &__key);			\
} while (0)

extern void __down_read(struct rw_semaphore *sem);
extern int __down_read_trylock(struct rw_semaphore *sem);
extern void __down_write(struct rw_semaphore *sem);
extern void __down_write_nested(struct rw_semaphore *sem, int subclass);
extern int __down_write_trylock(struct rw_semaphore *sem);
extern void __up_read(struct rw_semaphore *sem);
extern void __up_write(struct rw_semaphore *sem);
extern void __downgrade_write(struct rw_semaphore *sem);
extern int rwsem_is_locked(struct rw_semaphore *sem);

#endif /* __KERNEL__ */
#endif /* _LINUX_RWSEM_SPINLOCK_H */
