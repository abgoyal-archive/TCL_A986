
#ifndef _ALPHA_RESOURCE_H
#define _ALPHA_RESOURCE_H

#define RLIMIT_NOFILE		6	/* max number of open files */
#define RLIMIT_AS		7	/* address space limit */
#define RLIMIT_NPROC		8	/* max number of processes */
#define RLIMIT_MEMLOCK		9	/* max locked-in-memory address space */

#define RLIM_INFINITY		0x7ffffffffffffffful

#include <asm-generic/resource.h>

#endif /* _ALPHA_RESOURCE_H */
