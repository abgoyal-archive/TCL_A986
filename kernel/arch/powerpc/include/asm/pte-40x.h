
#ifndef _ASM_POWERPC_PTE_40x_H
#define _ASM_POWERPC_PTE_40x_H
#ifdef __KERNEL__


#define	_PAGE_GUARDED	0x001	/* G: page is guarded from prefetch */
#define _PAGE_FILE	0x001	/* when !present: nonlinear file mapping */
#define _PAGE_PRESENT	0x002	/* software: PTE contains a translation */
#define	_PAGE_NO_CACHE	0x004	/* I: caching is inhibited */
#define	_PAGE_WRITETHRU	0x008	/* W: caching is write-through */
#define	_PAGE_USER	0x010	/* matches one of the zone permission bits */
#define	_PAGE_SPECIAL	0x020	/* software: Special page */
#define	_PAGE_RW	0x040	/* software: Writes permitted */
#define	_PAGE_DIRTY	0x080	/* software: dirty page */
#define _PAGE_HWWRITE	0x100	/* hardware: Dirty & RW, set in exception */
#define _PAGE_EXEC	0x200	/* hardware: EX permission */
#define _PAGE_ACCESSED	0x400	/* software: R: page referenced */

#define _PMD_PRESENT	0x400	/* PMD points to page of PTEs */
#define _PMD_BAD	0x802
#define _PMD_SIZE	0x0e0	/* size field, != 0 for large-page PMD entry */
#define _PMD_SIZE_4M	0x0c0
#define _PMD_SIZE_16M	0x0e0

#define PMD_PAGE_SIZE(pmdval)	(1024 << (((pmdval) & _PMD_SIZE) >> 4))

/* Until my rework is finished, 40x still needs atomic PTE updates */
#define PTE_ATOMIC_UPDATES	1

#endif /* __KERNEL__ */
#endif /*  _ASM_POWERPC_PTE_40x_H */
