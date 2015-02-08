
#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H

#define DAVINCI_SYSTEM_MODULE_BASE        0x01C40000

/* System control register offsets */
#define DM64XX_VDD3P3V_PWDN	0x48

#define IO_PHYS				0x01c00000UL
#define IO_OFFSET			0xfd000000 /* Virtual IO = 0xfec00000 */
#define IO_SIZE				0x00400000
#define IO_VIRT				(IO_PHYS + IO_OFFSET)
#define io_v2p(va)			((va) - IO_OFFSET)
#define __IO_ADDRESS(x)			((x) + IO_OFFSET)
#define IO_ADDRESS(pa)			IOMEM(__IO_ADDRESS(pa))

#ifdef __ASSEMBLER__
#define IOMEM(x)                	x
#else
#define IOMEM(x)                	((void __force __iomem *)(x))
#endif

#endif /* __ASM_ARCH_HARDWARE_H */
