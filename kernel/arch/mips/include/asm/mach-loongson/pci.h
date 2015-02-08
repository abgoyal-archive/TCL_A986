

#ifndef __ASM_MACH_LOONGSON_PCI_H_
#define __ASM_MACH_LOONGSON_PCI_H_

extern struct pci_ops loongson_pci_ops;

/* this is an offset from mips_io_port_base */
#define LOONGSON_PCI_IO_START	0x00004000UL

#ifdef CONFIG_CPU_SUPPORTS_ADDRWINCFG


/* the smallest LOONGSON_CPU_MEM_SRC can be 512M */
#define LOONGSON_CPU_MEM_SRC	0x40000000ul		/* 1G */
#define LOONGSON_PCI_MEM_DST	LOONGSON_CPU_MEM_SRC

#define LOONGSON_PCI_MEM_START	LOONGSON_PCI_MEM_DST
#define LOONGSON_PCI_MEM_END	(0x80000000ul-1)	/* 2G */

#define MMAP_CPUTOPCI_SIZE	(LOONGSON_PCI_MEM_END - \
					LOONGSON_PCI_MEM_START + 1)

#else	/* loongson2f/32bit & loongson2e */

/* this pci memory space is mapped by pcimap in pci.c */
#define LOONGSON_PCI_MEM_START	LOONGSON_PCILO1_BASE
#define LOONGSON_PCI_MEM_END	(LOONGSON_PCILO1_BASE + 0x04000000 * 2)
/* this is an offset from mips_io_port_base */
#define LOONGSON_PCI_IO_START	0x00004000UL

#endif	/* !CONFIG_CPU_SUPPORTS_ADDRWINCFG */

#endif /* !__ASM_MACH_LOONGSON_PCI_H_ */
