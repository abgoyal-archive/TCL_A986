

#ifndef _ASM_POWERPC_TCE_H
#define _ASM_POWERPC_TCE_H
#ifdef __KERNEL__

#include <asm/iommu.h>

#define TCE_VB  0
#define TCE_PCI 1

/* TCE page size is 4096 bytes (1 << 12) */

#define TCE_SHIFT	12
#define TCE_PAGE_SIZE	(1 << TCE_SHIFT)

#define TCE_ENTRY_SIZE		8		/* each TCE is 64 bits */

#define TCE_RPN_MASK		0xfffffffffful  /* 40-bit RPN (4K pages) */
#define TCE_RPN_SHIFT		12
#define TCE_VALID		0x800		/* TCE valid */
#define TCE_ALLIO		0x400		/* TCE valid for all lpars */
#define TCE_PCI_WRITE		0x2		/* write from PCI allowed */
#define TCE_PCI_READ		0x1		/* read from PCI allowed */
#define TCE_VB_WRITE		0x1		/* write from VB allowed */

#endif /* __KERNEL__ */
#endif /* _ASM_POWERPC_TCE_H */
