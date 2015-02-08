

#ifndef __ASM_ARCH_IO_H
#define __ASM_ARCH_IO_H

#define IO_SPACE_LIMIT 0xffffffff

/* No ISA or PCI bus on this machine. */
#define __io(a)		__typesafe_io(a)
#define __mem_pci(a)	(a)

#endif /* __ASM_ARCH_IO_H */
