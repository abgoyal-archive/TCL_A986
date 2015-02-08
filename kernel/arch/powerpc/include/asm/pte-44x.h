
#ifndef _ASM_POWERPC_PTE_44x_H
#define _ASM_POWERPC_PTE_44x_H
#ifdef __KERNEL__


#define _PAGE_PRESENT	0x00000001		/* S: PTE valid */
#define _PAGE_RW	0x00000002		/* S: Write permission */
#define _PAGE_FILE	0x00000004		/* S: nonlinear file mapping */
#define _PAGE_EXEC	0x00000004		/* H: Execute permission */
#define _PAGE_ACCESSED	0x00000008		/* S: Page referenced */
#define _PAGE_DIRTY	0x00000010		/* S: Page dirty */
#define _PAGE_SPECIAL	0x00000020		/* S: Special page */
#define _PAGE_USER	0x00000040		/* S: User page */
#define _PAGE_ENDIAN	0x00000080		/* H: E bit */
#define _PAGE_GUARDED	0x00000100		/* H: G bit */
#define _PAGE_COHERENT	0x00000200		/* H: M bit */
#define _PAGE_NO_CACHE	0x00000400		/* H: I bit */
#define _PAGE_WRITETHRU	0x00000800		/* H: W bit */

/* TODO: Add large page lowmem mapping support */
#define _PMD_PRESENT	0
#define _PMD_PRESENT_MASK (PAGE_MASK)
#define _PMD_BAD	(~PAGE_MASK)

/* ERPN in a PTE never gets cleared, ignore it */
#define _PTE_NONE_MASK	0xffffffff00000000ULL


#endif /* __KERNEL__ */
#endif /*  _ASM_POWERPC_PTE_44x_H */
