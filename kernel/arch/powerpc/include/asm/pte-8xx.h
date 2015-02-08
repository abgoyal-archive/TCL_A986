
#ifndef _ASM_POWERPC_PTE_8xx_H
#define _ASM_POWERPC_PTE_8xx_H
#ifdef __KERNEL__


/* Definitions for 8xx embedded chips. */
#define _PAGE_PRESENT	0x0001	/* Page is valid */
#define _PAGE_FILE	0x0002	/* when !present: nonlinear file mapping */
#define _PAGE_NO_CACHE	0x0002	/* I: cache inhibit */
#define _PAGE_SHARED	0x0004	/* No ASID (context) compare */
#define _PAGE_SPECIAL	0x0008	/* SW entry, forced to 0 by the TLB miss */
#define _PAGE_DIRTY	0x0100	/* C: page changed */

#define _PAGE_GUARDED	0x0010	/* software: guarded access */
#define _PAGE_ACCESSED	0x0020	/* software: page referenced */
#define _PAGE_WRITETHRU	0x0040	/* software: caching is write through */

#define _PAGE_RW	0x0400	/* lsb PP bits, inverted in HW */
#define _PAGE_USER	0x0800	/* msb PP bits */

#define _PMD_PRESENT	0x0001
#define _PMD_BAD	0x0ff0
#define _PMD_PAGE_MASK	0x000c
#define _PMD_PAGE_8M	0x000c

#define _PTE_NONE_MASK _PAGE_ACCESSED

/* Until my rework is finished, 8xx still needs atomic PTE updates */
#define PTE_ATOMIC_UPDATES	1

/* We need to add _PAGE_SHARED to kernel pages */
#define _PAGE_KERNEL_RO	(_PAGE_SHARED)
#define _PAGE_KERNEL_RW	(_PAGE_DIRTY | _PAGE_RW | _PAGE_HWWRITE)

#endif /* __KERNEL__ */
#endif /*  _ASM_POWERPC_PTE_8xx_H */
