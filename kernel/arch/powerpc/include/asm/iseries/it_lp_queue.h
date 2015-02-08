
#ifndef _ASM_POWERPC_ISERIES_IT_LP_QUEUE_H
#define _ASM_POWERPC_ISERIES_IT_LP_QUEUE_H


#include <asm/types.h>
#include <asm/ptrace.h>

#define IT_LP_MAX_QUEUES	8

#define IT_LP_NOT_USED		0	/* Queue will not be used by PLIC */
#define IT_LP_DEDICATED_IO	1	/* Queue dedicated to IO processor specified */
#define IT_LP_DEDICATED_LP	2	/* Queue dedicated to LP specified */
#define IT_LP_SHARED		3	/* Queue shared for both IO and LP */

#define IT_LP_EVENT_STACK_SIZE	4096
#define IT_LP_EVENT_MAX_SIZE	256
#define IT_LP_EVENT_ALIGN	64

struct hvlpevent_queue {
	u8		hq_overflow_pending;	/* 0x00 Overflow events are pending */
	u8		hq_status;		/* 0x01 DedicatedIo or DedicatedLp or NotUsed */
	u16		hq_proc_index;		/* 0x02 Logical Proc Index for correlation */
	u8		hq_reserved1[12];	/* 0x04 */
	char		*hq_current_event;	/* 0x10 */
	char		*hq_last_event;		/* 0x18 */
	char		*hq_event_stack;	/* 0x20 */
	u8		hq_index;		/* 0x28 unique sequential index. */
	u8		hq_reserved2[3];	/* 0x29-2b */
	spinlock_t	hq_lock;
};

extern struct hvlpevent_queue hvlpevent_queue;

extern int hvlpevent_is_pending(void);
extern void process_hvlpevents(void);
extern void setup_hvlpevent_queue(void);

#endif /* _ASM_POWERPC_ISERIES_IT_LP_QUEUE_H */
