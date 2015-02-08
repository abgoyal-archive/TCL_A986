

#ifndef _ASM_DM9000_H
#define _ASM_DM9000_H

#include <asm/mb-regs.h>

#define DM9000_ARCH_IOBASE	(__region_CS6 + 0x300)
#define DM9000_ARCH_IRQ		IRQ_CPU_EXTERNAL3	/* XIRQ #3 (shared with FPGA) */
#undef DM9000_ARCH_IRQ_ACTLOW				/* IRQ pin active high */
#define DM9000_ARCH_BUS_INFO	"CS6#+0x300"		/* bus info for ethtool */

#undef __is_PCI_IO
#define __is_PCI_IO(addr)	0	/* not PCI */

#undef inl
#define inl(addr)										\
({												\
	unsigned long __ioaddr = (unsigned long) addr;						\
	uint32_t x = readl(__ioaddr);								\
	((x & 0xff) << 24) | ((x & 0xff00) << 8) | ((x >> 8) & 0xff00) | ((x >> 24) & 0xff);	\
})

#undef insl
#define insl(a,b,l)	__insl(a,b,l,0) /* don't byte-swap */


#endif /* _ASM_DM9000_H */
