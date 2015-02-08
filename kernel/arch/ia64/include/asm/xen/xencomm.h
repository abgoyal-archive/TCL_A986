

#ifndef _ASM_IA64_XEN_XENCOMM_H
#define _ASM_IA64_XEN_XENCOMM_H

#include <xen/xencomm.h>
#include <asm/pgtable.h>

/* Must be called before any hypercall.  */
extern void xencomm_initialize(void);
extern int xencomm_is_initialized(void);

static inline int xencomm_is_phys_contiguous(unsigned long addr)
{
	return (PAGE_OFFSET <= addr &&
		addr < (PAGE_OFFSET + (1UL << IA64_MAX_PHYS_BITS))) ||
		(KERNEL_START <= addr &&
		 addr < KERNEL_START + KERNEL_TR_PAGE_SIZE);
}

#endif /* _ASM_IA64_XEN_XENCOMM_H */
