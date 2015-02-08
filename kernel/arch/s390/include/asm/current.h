

#ifndef _S390_CURRENT_H
#define _S390_CURRENT_H

#ifdef __KERNEL__
#include <asm/lowcore.h>

struct task_struct;

#define current ((struct task_struct *const)S390_lowcore.current_task)

#endif

#endif /* !(_S390_CURRENT_H) */
