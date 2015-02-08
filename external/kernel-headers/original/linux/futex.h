#ifndef _LINUX_FUTEX_H
#define _LINUX_FUTEX_H

#include <linux/sched.h>

/* Second argument to futex syscall */


#define FUTEX_WAIT		0
#define FUTEX_WAKE		1
#define FUTEX_FD		2
#define FUTEX_REQUEUE		3
#define FUTEX_CMP_REQUEUE	4
#define FUTEX_WAKE_OP		5
#define FUTEX_LOCK_PI		6
#define FUTEX_UNLOCK_PI		7
#define FUTEX_TRYLOCK_PI	8


struct robust_list {
	struct robust_list __user *next;
};

struct robust_list_head {
	/*
	 * The head of the list. Points back to itself if empty:
	 */
	struct robust_list list;

	/*
	 * This relative offset is set by user-space, it gives the kernel
	 * the relative position of the futex field to examine. This way
	 * we keep userspace flexible, to freely shape its data-structure,
	 * without hardcoding any particular offset into the kernel:
	 */
	long futex_offset;

	/*
	 * The death of the thread may race with userspace setting
	 * up a lock's links. So to handle this race, userspace first
	 * sets this field to the address of the to-be-taken lock,
	 * then does the lock acquire, and then adds itself to the
	 * list, and then clears this field. Hence the kernel will
	 * always have full knowledge of all locks that the thread
	 * _might_ have taken. We check the owner TID in any case,
	 * so only truly owned locks will be handled.
	 */
	struct robust_list __user *list_op_pending;
};

#define FUTEX_WAITERS		0x80000000

#define FUTEX_OWNER_DIED	0x40000000

#define FUTEX_TID_MASK		0x3fffffff

#define ROBUST_LIST_LIMIT	2048

long do_futex(u32 __user *uaddr, int op, u32 val, unsigned long timeout,
	      u32 __user *uaddr2, u32 val2, u32 val3);

extern int
handle_futex_death(u32 __user *uaddr, struct task_struct *curr, int pi);

#ifdef CONFIG_FUTEX
extern void exit_robust_list(struct task_struct *curr);
extern void exit_pi_state_list(struct task_struct *curr);
#else
static inline void exit_robust_list(struct task_struct *curr)
{
}
static inline void exit_pi_state_list(struct task_struct *curr)
{
}
#endif

#define FUTEX_OP_SET		0	/* *(int *)UADDR2 = OPARG; */
#define FUTEX_OP_ADD		1	/* *(int *)UADDR2 += OPARG; */
#define FUTEX_OP_OR		2	/* *(int *)UADDR2 |= OPARG; */
#define FUTEX_OP_ANDN		3	/* *(int *)UADDR2 &= ~OPARG; */
#define FUTEX_OP_XOR		4	/* *(int *)UADDR2 ^= OPARG; */

#define FUTEX_OP_OPARG_SHIFT	8	/* Use (1 << OPARG) instead of OPARG.  */

#define FUTEX_OP_CMP_EQ		0	/* if (oldval == CMPARG) wake */
#define FUTEX_OP_CMP_NE		1	/* if (oldval != CMPARG) wake */
#define FUTEX_OP_CMP_LT		2	/* if (oldval < CMPARG) wake */
#define FUTEX_OP_CMP_LE		3	/* if (oldval <= CMPARG) wake */
#define FUTEX_OP_CMP_GT		4	/* if (oldval > CMPARG) wake */
#define FUTEX_OP_CMP_GE		5	/* if (oldval >= CMPARG) wake */


#define FUTEX_OP(op, oparg, cmp, cmparg) \
  (((op & 0xf) << 28) | ((cmp & 0xf) << 24)		\
   | ((oparg & 0xfff) << 12) | (cmparg & 0xfff))

#endif
