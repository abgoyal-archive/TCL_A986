

#ifndef _SPARC_RESOURCE_H
#define _SPARC_RESOURCE_H

#define RLIMIT_NOFILE		6	/* max number of open files */
#define RLIMIT_NPROC		7	/* max number of processes */

#if defined(__sparc__) && defined(__arch64__)
/* Use generic version */
#else
#define RLIM_INFINITY		0x7fffffff
#endif

#include <asm-generic/resource.h>

#endif /* !(_SPARC_RESOURCE_H) */
