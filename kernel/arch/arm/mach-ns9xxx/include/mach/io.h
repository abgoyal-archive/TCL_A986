
#ifndef __ASM_ARCH_IO_H
#define __ASM_ARCH_IO_H

#define IO_SPACE_LIMIT  0xffffffff /* XXX */

#define __io(a)		__typesafe_io(a)
#define __mem_pci(a)    (a)
#define __mem_isa(a)    (IO_BASE + (a))

#endif /* ifndef __ASM_ARCH_IO_H */
