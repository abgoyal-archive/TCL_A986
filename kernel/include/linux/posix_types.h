
#ifndef _LINUX_POSIX_TYPES_H
#define _LINUX_POSIX_TYPES_H

#include <linux/stddef.h>


#undef __NFDBITS
#define __NFDBITS	(8 * sizeof(unsigned long))

#undef __FD_SETSIZE
#define __FD_SETSIZE	1024

#undef __FDSET_LONGS
#define __FDSET_LONGS	(__FD_SETSIZE/__NFDBITS)

#undef __FDELT
#define	__FDELT(d)	((d) / __NFDBITS)

#undef __FDMASK
#define	__FDMASK(d)	(1UL << ((d) % __NFDBITS))

typedef struct {
	unsigned long fds_bits [__FDSET_LONGS];
} __kernel_fd_set;

/* Type of a signal handler.  */
typedef void (*__kernel_sighandler_t)(int);

/* Type of a SYSV IPC key.  */
typedef int __kernel_key_t;
typedef int __kernel_mqd_t;

#include <asm/posix_types.h>

#endif /* _LINUX_POSIX_TYPES_H */
