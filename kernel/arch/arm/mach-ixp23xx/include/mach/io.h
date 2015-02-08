

#ifndef __ASM_ARCH_IO_H
#define __ASM_ARCH_IO_H

#define IO_SPACE_LIMIT 0xffffffff

#define __io(p)		((void __iomem*)((p) + IXP23XX_PCI_IO_VIRT))
#define __mem_pci(a)	(a)

static inline void __iomem *
ixp23xx_ioremap(unsigned long addr, unsigned long size, unsigned int mtype)
{
	if (addr >= IXP23XX_PCI_MEM_START &&
		addr <= IXP23XX_PCI_MEM_START + IXP23XX_PCI_MEM_SIZE) {
		if (addr + size > IXP23XX_PCI_MEM_START + IXP23XX_PCI_MEM_SIZE)
			return NULL;

		return (void __iomem *)
 			((addr - IXP23XX_PCI_MEM_START) + IXP23XX_PCI_MEM_VIRT);
	}

	return __arm_ioremap(addr, size, mtype);
}

static inline void
ixp23xx_iounmap(void __iomem *addr)
{
	if ((((u32)addr) >= IXP23XX_PCI_MEM_VIRT) &&
	    (((u32)addr) < IXP23XX_PCI_MEM_VIRT + IXP23XX_PCI_MEM_SIZE))
		return;

	__iounmap(addr);
}

#define __arch_ioremap(a,s,f)	ixp23xx_ioremap(a,s,f)
#define __arch_iounmap(a)	ixp23xx_iounmap(a)


#endif
