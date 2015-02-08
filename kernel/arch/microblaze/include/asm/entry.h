

#ifndef _ASM_MICROBLAZE_ENTRY_H
#define _ASM_MICROBLAZE_ENTRY_H

#include <asm/percpu.h>
#include <asm/ptrace.h>


#define PER_CPU(var) var

# ifndef __ASSEMBLY__
DECLARE_PER_CPU(unsigned int, KSP); /* Saved kernel stack pointer */
DECLARE_PER_CPU(unsigned int, KM); /* Kernel/user mode */
DECLARE_PER_CPU(unsigned int, ENTRY_SP); /* Saved SP on kernel entry */
DECLARE_PER_CPU(unsigned int, R11_SAVE); /* Temp variable for entry */
DECLARE_PER_CPU(unsigned int, CURRENT_SAVE); /* Saved current pointer */
# endif /* __ASSEMBLY__ */

#ifndef CONFIG_MMU

/* noMMU hasn't any space for args */
# define STATE_SAVE_ARG_SPACE	(0)

#else /* CONFIG_MMU */



#define STATE_SAVE_ARG_SPACE	(6*4) /* Up to six arguments */

#endif /* CONFIG_MMU */

#endif /* _ASM_MICROBLAZE_ENTRY_H */
