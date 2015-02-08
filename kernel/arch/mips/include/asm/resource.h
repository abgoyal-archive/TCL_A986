
#ifndef _ASM_RESOURCE_H
#define _ASM_RESOURCE_H


#define RLIMIT_NOFILE		5	/* max number of open files */
#define RLIMIT_AS		6	/* address space limit */
#define RLIMIT_RSS		7	/* max resident set size */
#define RLIMIT_NPROC		8	/* max number of processes */
#define RLIMIT_MEMLOCK		9	/* max locked-in-memory address space */

#ifdef CONFIG_32BIT
# define RLIM_INFINITY		0x7fffffffUL
#endif

#include <asm-generic/resource.h>

#endif /* _ASM_RESOURCE_H */
