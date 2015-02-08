
#ifndef __ASM_AVR32_ADDRSPACE_H
#define __ASM_AVR32_ADDRSPACE_H

#ifdef CONFIG_MMU

/* Memory segments when segmentation is enabled */
#define P0SEG		0x00000000
#define P1SEG		0x80000000
#define P2SEG		0xa0000000
#define P3SEG		0xc0000000
#define P4SEG		0xe0000000

/* Returns the privileged segment base of a given address */
#define PXSEG(a)	(((unsigned long)(a)) & 0xe0000000)

/* Returns the physical address of a PnSEG (n=1,2) address */
#define PHYSADDR(a)	(((unsigned long)(a)) & 0x1fffffff)

#define P1SEGADDR(a) ((__typeof__(a))(((unsigned long)(a) & 0x1fffffff) \
				      | P1SEG))
#define P2SEGADDR(a) ((__typeof__(a))(((unsigned long)(a) & 0x1fffffff) \
				      | P2SEG))
#define P3SEGADDR(a) ((__typeof__(a))(((unsigned long)(a) & 0x1fffffff) \
				      | P3SEG))
#define P4SEGADDR(a) ((__typeof__(a))(((unsigned long)(a) & 0x1fffffff) \
				      | P4SEG))

#endif /* CONFIG_MMU */

#endif /* __ASM_AVR32_ADDRSPACE_H */
