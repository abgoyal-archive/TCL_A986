

#ifndef _S390_IO_H
#define _S390_IO_H

#ifdef __KERNEL__

#include <asm/page.h>

#define IO_SPACE_LIMIT 0xffffffff

static inline unsigned long virt_to_phys(volatile void * address)
{
	unsigned long real_address;
	asm volatile(
		 "	lra	%0,0(%1)\n"
		 "	jz	0f\n"
		 "	la	%0,0\n"
                 "0:"
		 : "=a" (real_address) : "a" (address) : "cc");
        return real_address;
}

static inline void * phys_to_virt(unsigned long address)
{
	return (void *) address;
}

#define xlate_dev_mem_ptr(p)	__va(p)

#define xlate_dev_kmem_ptr(p)	p

#endif /* __KERNEL__ */

#endif
