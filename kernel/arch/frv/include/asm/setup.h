

#ifndef _ASM_SETUP_H
#define _ASM_SETUP_H

#define COMMAND_LINE_SIZE       512

#ifdef __KERNEL__

#include <linux/init.h>

#ifndef __ASSEMBLY__

#ifdef CONFIG_MMU
extern unsigned long __initdata num_mappedpages;
#endif

#endif /* !__ASSEMBLY__ */

#endif  /*  __KERNEL__  */

#endif /* _ASM_SETUP_H */
