#ifndef __LINUX_LOCKDEP_H
#define __LINUX_LOCKDEP_H

#include <linux/linkage.h>
#include <linux/list.h>
#include <linux/debug_locks.h>
#include <linux/stacktrace.h>

#ifdef CONFIG_LOCKDEP

enum lock_usage_bit
{
	LOCK_USED = 0,
	LOCK_USED_IN_HARDIRQ,
	LOCK_USED_IN_SOFTIRQ,
	LOCK_ENABLED_SOFTIRQS,
	LOCK_ENABLED_HARDIRQS,
	LOCK_USED_IN_HARDIRQ_READ,
	LOCK_USED_IN_SOFTIRQ_READ,
	LOCK_ENABLED_SOFTIRQS_READ,
	LOCK_ENABLED_HARDIRQS_READ,
	LOCK_USAGE_STATES
};

#define LOCKF_USED			(1 << LOCK_USED)
#define LOCKF_USED_IN_HARDIRQ		(1 << LOCK_USED_IN_HARDIRQ)
#define LOCKF_USED_IN_SOFTIRQ		(1 << LOCK_USED_IN_SOFTIRQ)
#define LOCKF_ENABLED_HARDIRQS		(1 << LOCK_ENABLED_HARDIRQS)
#define LOCKF_ENABLED_SOFTIRQS		(1 << LOCK_ENABLED_SOFTIRQS)

#define LOCKF_ENABLED_IRQS (LOCKF_ENABLED_HARDIRQS | LOCKF_ENABLED_SOFTIRQS)
#define LOCKF_USED_IN_IRQ (LOCKF_USED_IN_HARDIRQ | LOCKF_USED_IN_SOFTIRQ)

#define LOCKF_USED_IN_HARDIRQ_READ	(1 << LOCK_USED_IN_HARDIRQ_READ)
#define LOCKF_USED_IN_SOFTIRQ_READ	(1 << LOCK_USED_IN_SOFTIRQ_READ)
#define LOCKF_ENABLED_HARDIRQS_READ	(1 << LOCK_ENABLED_HARDIRQS_READ)
#define LOCKF_ENABLED_SOFTIRQS_READ	(1 << LOCK_ENABLED_SOFTIRQS_READ)

#define LOCKF_ENABLED_IRQS_READ \
		(LOCKF_ENABLED_HARDIRQS_READ | LOCKF_ENABLED_SOFTIRQS_READ)
#define LOCKF_USED_IN_IRQ_READ \
		(LOCKF_USED_IN_HARDIRQ_READ | LOCKF_USED_IN_SOFTIRQ_READ)

#define MAX_LOCKDEP_SUBCLASSES		8UL

struct lockdep_subclass_key {
	char __one_byte;
} __attribute__ ((__packed__));

struct lock_class_key {
	struct lockdep_subclass_key	subkeys[MAX_LOCKDEP_SUBCLASSES];
};

struct lock_class {
	/*
	 * class-hash:
	 */
	struct list_head		hash_entry;

	/*
	 * global list of all lock-classes:
	 */
	struct list_head		lock_entry;

	struct lockdep_subclass_key	*key;
	unsigned int			subclass;

	/*
	 * IRQ/softirq usage tracking bits:
	 */
	unsigned long			usage_mask;
	struct stack_trace		usage_traces[LOCK_USAGE_STATES];

	/*
	 * These fields represent a directed graph of lock dependencies,
	 * to every node we attach a list of "forward" and a list of
	 * "backward" graph nodes.
	 */
	struct list_head		locks_after, locks_before;

	/*
	 * Generation counter, when doing certain classes of graph walking,
	 * to ensure that we check one node only once:
	 */
	unsigned int			version;

	/*
	 * Statistics counter:
	 */
	unsigned long			ops;

	const char			*name;
	int				name_version;
};

struct lockdep_map {
	struct lock_class_key		*key;
	struct lock_class		*class_cache;
	const char			*name;
};

struct lock_list {
	struct list_head		entry;
	struct lock_class		*class;
	struct stack_trace		trace;
};

struct lock_chain {
	struct list_head		entry;
	u64				chain_key;
};

struct held_lock {
	/*
	 * One-way hash of the dependency chain up to this point. We
	 * hash the hashes step by step as the dependency chain grows.
	 *
	 * We use it for dependency-caching and we skip detection
	 * passes and dependency-updates if there is a cache-hit, so
	 * it is absolutely critical for 100% coverage of the validator
	 * to have a unique key value for every unique dependency path
	 * that can occur in the system, to make a unique hash value
	 * as likely as possible - hence the 64-bit width.
	 *
	 * The task struct holds the current hash value (initialized
	 * with zero), here we store the previous hash value:
	 */
	u64				prev_chain_key;
	struct lock_class		*class;
	unsigned long			acquire_ip;
	struct lockdep_map		*instance;

	/*
	 * The lock-stack is unified in that the lock chains of interrupt
	 * contexts nest ontop of process context chains, but we 'separate'
	 * the hashes by starting with 0 if we cross into an interrupt
	 * context, and we also keep do not add cross-context lock
	 * dependencies - the lock usage graph walking covers that area
	 * anyway, and we'd just unnecessarily increase the number of
	 * dependencies otherwise. [Note: hardirq and softirq contexts
	 * are separated from each other too.]
	 *
	 * The following field is used to detect when we cross into an
	 * interrupt context:
	 */
	int				irq_context;
	int				trylock;
	int				read;
	int				check;
	int				hardirqs_off;
};

extern void lockdep_init(void);
extern void lockdep_info(void);
extern void lockdep_reset(void);
extern void lockdep_reset_lock(struct lockdep_map *lock);
extern void lockdep_free_key_range(void *start, unsigned long size);

extern void lockdep_off(void);
extern void lockdep_on(void);
extern int lockdep_internal(void);


extern void lockdep_init_map(struct lockdep_map *lock, const char *name,
			     struct lock_class_key *key);

#define lockdep_set_class(lock, key) \
		lockdep_init_map(&(lock)->dep_map, #key, key)
#define lockdep_set_class_and_name(lock, key, name) \
		lockdep_init_map(&(lock)->dep_map, name, key)

extern void lock_acquire(struct lockdep_map *lock, unsigned int subclass,
			 int trylock, int read, int check, unsigned long ip);

extern void lock_release(struct lockdep_map *lock, int nested,
			 unsigned long ip);

# define INIT_LOCKDEP				.lockdep_recursion = 0,

#else /* !LOCKDEP */

static inline void lockdep_off(void)
{
}

static inline void lockdep_on(void)
{
}

static inline int lockdep_internal(void)
{
	return 0;
}

# define lock_acquire(l, s, t, r, c, i)		do { } while (0)
# define lock_release(l, n, i)			do { } while (0)
# define lockdep_init()				do { } while (0)
# define lockdep_info()				do { } while (0)
# define lockdep_init_map(lock, name, key)	do { (void)(key); } while (0)
# define lockdep_set_class(lock, key)		do { (void)(key); } while (0)
# define lockdep_set_class_and_name(lock, key, name) \
		do { (void)(key); } while (0)
# define INIT_LOCKDEP
# define lockdep_reset()		do { debug_locks = 1; } while (0)
# define lockdep_free_key_range(start, size)	do { } while (0)
struct lock_class_key { };
#endif /* !LOCKDEP */

#if defined(CONFIG_TRACE_IRQFLAGS) && defined(CONFIG_GENERIC_HARDIRQS)
extern void early_init_irq_lock_class(void);
#else
# define early_init_irq_lock_class()		do { } while (0)
#endif

#ifdef CONFIG_TRACE_IRQFLAGS
extern void early_boot_irqs_off(void);
extern void early_boot_irqs_on(void);
#else
# define early_boot_irqs_off()			do { } while (0)
# define early_boot_irqs_on()			do { } while (0)
#endif

#define SINGLE_DEPTH_NESTING			1


#ifdef CONFIG_DEBUG_LOCK_ALLOC
# ifdef CONFIG_PROVE_LOCKING
#  define spin_acquire(l, s, t, i)		lock_acquire(l, s, t, 0, 2, i)
# else
#  define spin_acquire(l, s, t, i)		lock_acquire(l, s, t, 0, 1, i)
# endif
# define spin_release(l, n, i)			lock_release(l, n, i)
#else
# define spin_acquire(l, s, t, i)		do { } while (0)
# define spin_release(l, n, i)			do { } while (0)
#endif

#ifdef CONFIG_DEBUG_LOCK_ALLOC
# ifdef CONFIG_PROVE_LOCKING
#  define rwlock_acquire(l, s, t, i)		lock_acquire(l, s, t, 0, 2, i)
#  define rwlock_acquire_read(l, s, t, i)	lock_acquire(l, s, t, 2, 2, i)
# else
#  define rwlock_acquire(l, s, t, i)		lock_acquire(l, s, t, 0, 1, i)
#  define rwlock_acquire_read(l, s, t, i)	lock_acquire(l, s, t, 2, 1, i)
# endif
# define rwlock_release(l, n, i)		lock_release(l, n, i)
#else
# define rwlock_acquire(l, s, t, i)		do { } while (0)
# define rwlock_acquire_read(l, s, t, i)	do { } while (0)
# define rwlock_release(l, n, i)		do { } while (0)
#endif

#ifdef CONFIG_DEBUG_LOCK_ALLOC
# ifdef CONFIG_PROVE_LOCKING
#  define mutex_acquire(l, s, t, i)		lock_acquire(l, s, t, 0, 2, i)
# else
#  define mutex_acquire(l, s, t, i)		lock_acquire(l, s, t, 0, 1, i)
# endif
# define mutex_release(l, n, i)			lock_release(l, n, i)
#else
# define mutex_acquire(l, s, t, i)		do { } while (0)
# define mutex_release(l, n, i)			do { } while (0)
#endif

#ifdef CONFIG_DEBUG_LOCK_ALLOC
# ifdef CONFIG_PROVE_LOCKING
#  define rwsem_acquire(l, s, t, i)		lock_acquire(l, s, t, 0, 2, i)
#  define rwsem_acquire_read(l, s, t, i)	lock_acquire(l, s, t, 1, 2, i)
# else
#  define rwsem_acquire(l, s, t, i)		lock_acquire(l, s, t, 0, 1, i)
#  define rwsem_acquire_read(l, s, t, i)	lock_acquire(l, s, t, 1, 1, i)
# endif
# define rwsem_release(l, n, i)			lock_release(l, n, i)
#else
# define rwsem_acquire(l, s, t, i)		do { } while (0)
# define rwsem_acquire_read(l, s, t, i)		do { } while (0)
# define rwsem_release(l, n, i)			do { } while (0)
#endif

#endif /* __LINUX_LOCKDEP_H */
