

#ifndef _L1LAYOUT_H_
#define _L1LAYOUT_H_

#include <asm/blackfin.h>

#ifndef CONFIG_SMP
#ifndef __ASSEMBLY__

struct l1_scratch_task_info
{
	/* Points to the start of the stack.  */
	void *stack_start;
	/* Not updated by the kernel; a user process can modify this to
	   keep track of the lowest address of the stack pointer during its
	   runtime.  */
	void *lowest_sp;
};

/* A pointer to the structure in memory.  */
#define L1_SCRATCH_TASK_INFO ((struct l1_scratch_task_info *)\
						get_l1_scratch_start())

#endif
#endif

#endif
