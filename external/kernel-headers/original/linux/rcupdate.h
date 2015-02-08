
#ifndef __LINUX_RCUPDATE_H
#define __LINUX_RCUPDATE_H

#ifdef __KERNEL__

#include <linux/cache.h>
#include <linux/spinlock.h>
#include <linux/threads.h>
#include <linux/percpu.h>
#include <linux/cpumask.h>
#include <linux/seqlock.h>

struct rcu_head {
	struct rcu_head *next;
	void (*func)(struct rcu_head *head);
};

#define RCU_HEAD_INIT 	{ .next = NULL, .func = NULL }
#define RCU_HEAD(head) struct rcu_head head = RCU_HEAD_INIT
#define INIT_RCU_HEAD(ptr) do { \
       (ptr)->next = NULL; (ptr)->func = NULL; \
} while (0)



/* Global control variables for rcupdate callback mechanism. */
struct rcu_ctrlblk {
	long	cur;		/* Current batch number.                      */
	long	completed;	/* Number of the last completed batch         */
	int	next_pending;	/* Is the next batch already waiting?         */

	spinlock_t	lock	____cacheline_internodealigned_in_smp;
	cpumask_t	cpumask; /* CPUs that need to switch in order    */
	                         /* for current batch to proceed.        */
} ____cacheline_internodealigned_in_smp;

/* Is batch a before batch b ? */
static inline int rcu_batch_before(long a, long b)
{
        return (a - b) < 0;
}

/* Is batch a after batch b ? */
static inline int rcu_batch_after(long a, long b)
{
        return (a - b) > 0;
}

struct rcu_data {
	/* 1) quiescent state handling : */
	long		quiescbatch;     /* Batch # for grace period */
	int		passed_quiesc;	 /* User-mode/idle loop etc. */
	int		qs_pending;	 /* core waits for quiesc state */

	/* 2) batch handling */
	long  	       	batch;           /* Batch # for current RCU batch */
	struct rcu_head *nxtlist;
	struct rcu_head **nxttail;
	long            qlen; 	 	 /* # of queued callbacks */
	struct rcu_head *curlist;
	struct rcu_head **curtail;
	struct rcu_head *donelist;
	struct rcu_head **donetail;
	long		blimit;		 /* Upper limit on a processed batch */
	int cpu;
	struct rcu_head barrier;
#ifdef CONFIG_SMP
	long		last_rs_qlen;	 /* qlen during the last resched */
#endif
};

DECLARE_PER_CPU(struct rcu_data, rcu_data);
DECLARE_PER_CPU(struct rcu_data, rcu_bh_data);

static inline void rcu_qsctr_inc(int cpu)
{
	struct rcu_data *rdp = &per_cpu(rcu_data, cpu);
	rdp->passed_quiesc = 1;
}
static inline void rcu_bh_qsctr_inc(int cpu)
{
	struct rcu_data *rdp = &per_cpu(rcu_bh_data, cpu);
	rdp->passed_quiesc = 1;
}

extern int rcu_pending(int cpu);
extern int rcu_needs_cpu(int cpu);

#define rcu_read_lock() \
	do { \
		preempt_disable(); \
		__acquire(RCU); \
	} while(0)

#define rcu_read_unlock() \
	do { \
		__release(RCU); \
		preempt_enable(); \
	} while(0)


#define rcu_read_lock_bh() \
	do { \
		local_bh_disable(); \
		__acquire(RCU_BH); \
	} while(0)

#define rcu_read_unlock_bh() \
	do { \
		__release(RCU_BH); \
		local_bh_enable(); \
	} while(0)


#define rcu_dereference(p)     ({ \
				typeof(p) _________p1 = p; \
				smp_read_barrier_depends(); \
				(_________p1); \
				})


#define rcu_assign_pointer(p, v)	({ \
						smp_wmb(); \
						(p) = (v); \
					})

#define synchronize_sched() synchronize_rcu()

extern void rcu_init(void);
extern void rcu_check_callbacks(int cpu, int user);
extern void rcu_restart_cpu(int cpu);
extern long rcu_batches_completed(void);
extern long rcu_batches_completed_bh(void);

/* Exported interfaces */
extern void FASTCALL(call_rcu(struct rcu_head *head, 
				void (*func)(struct rcu_head *head)));
extern void FASTCALL(call_rcu_bh(struct rcu_head *head,
				void (*func)(struct rcu_head *head)));
extern void synchronize_rcu(void);
void synchronize_idle(void);
extern void rcu_barrier(void);

#endif /* __KERNEL__ */
#endif /* __LINUX_RCUPDATE_H */
