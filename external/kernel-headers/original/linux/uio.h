#ifndef __LINUX_UIO_H
#define __LINUX_UIO_H

#include <linux/compiler.h>
#include <linux/types.h>




struct iovec
{
	void __user *iov_base;	/* BSD uses caddr_t (1003.1g requires void *) */
	__kernel_size_t iov_len; /* Must be size_t (1003.1g) */
};

#ifdef __KERNEL__

struct kvec {
	void *iov_base; /* and that should *never* hold a userland pointer */
	size_t iov_len;
};

#endif

 
#define UIO_FASTIOV	8
#define UIO_MAXIOV	1024
#if 0
#define UIO_MAXIOV	16	/* Maximum iovec's in one operation 
				   16 matches BSD */
                                /* Beg pardon: BSD has 1024 --ANK */
#endif

static inline size_t iov_length(const struct iovec *iov, unsigned long nr_segs)
{
	unsigned long seg;
	size_t ret = 0;

	for (seg = 0; seg < nr_segs; seg++)
		ret += iov[seg].iov_len;
	return ret;
}

unsigned long iov_shorten(struct iovec *iov, unsigned long nr_segs, size_t to);

#endif
