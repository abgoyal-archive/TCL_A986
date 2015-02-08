

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/rwsem.h>

#include <asm/system.h>
#include <asm/atomic.h>

void __sched down_read(struct rw_semaphore *sem)
{
	might_sleep();
	rwsem_acquire_read(&sem->dep_map, 0, 0, _RET_IP_);

	LOCK_CONTENDED(sem, __down_read_trylock, __down_read);
}

EXPORT_SYMBOL(down_read);

int down_read_trylock(struct rw_semaphore *sem)
{
	int ret = __down_read_trylock(sem);

	if (ret == 1)
		rwsem_acquire_read(&sem->dep_map, 0, 1, _RET_IP_);
	return ret;
}

EXPORT_SYMBOL(down_read_trylock);

void __sched down_write(struct rw_semaphore *sem)
{
	might_sleep();
	rwsem_acquire(&sem->dep_map, 0, 0, _RET_IP_);

	LOCK_CONTENDED(sem, __down_write_trylock, __down_write);
}

EXPORT_SYMBOL(down_write);

int down_write_trylock(struct rw_semaphore *sem)
{
	int ret = __down_write_trylock(sem);

	if (ret == 1)
		rwsem_acquire(&sem->dep_map, 0, 1, _RET_IP_);
	return ret;
}

EXPORT_SYMBOL(down_write_trylock);

void up_read(struct rw_semaphore *sem)
{
	rwsem_release(&sem->dep_map, 1, _RET_IP_);

	__up_read(sem);
}

EXPORT_SYMBOL(up_read);

void up_write(struct rw_semaphore *sem)
{
	rwsem_release(&sem->dep_map, 1, _RET_IP_);

	__up_write(sem);
}

EXPORT_SYMBOL(up_write);

void downgrade_write(struct rw_semaphore *sem)
{
	/*
	 * lockdep: a downgraded write will live on as a write
	 * dependency.
	 */
	__downgrade_write(sem);
}

EXPORT_SYMBOL(downgrade_write);

#ifdef CONFIG_DEBUG_LOCK_ALLOC

void down_read_nested(struct rw_semaphore *sem, int subclass)
{
	might_sleep();
	rwsem_acquire_read(&sem->dep_map, subclass, 0, _RET_IP_);

	LOCK_CONTENDED(sem, __down_read_trylock, __down_read);
}

EXPORT_SYMBOL(down_read_nested);

void down_read_non_owner(struct rw_semaphore *sem)
{
	might_sleep();

	__down_read(sem);
}

EXPORT_SYMBOL(down_read_non_owner);

void down_write_nested(struct rw_semaphore *sem, int subclass)
{
	might_sleep();
	rwsem_acquire(&sem->dep_map, subclass, 0, _RET_IP_);

	LOCK_CONTENDED(sem, __down_write_trylock, __down_write);
}

EXPORT_SYMBOL(down_write_nested);

void up_read_non_owner(struct rw_semaphore *sem)
{
	__up_read(sem);
}

EXPORT_SYMBOL(up_read_non_owner);

#endif


