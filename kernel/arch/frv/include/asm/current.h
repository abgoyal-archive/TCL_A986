

#ifndef _ASM_CURRENT_H
#define _ASM_CURRENT_H

#ifndef __ASSEMBLY__

register struct task_struct *current asm("gr29");

#define get_current() current

#else

#define CURRENT gr29

#endif

#endif /* _ASM_CURRENT_H */
