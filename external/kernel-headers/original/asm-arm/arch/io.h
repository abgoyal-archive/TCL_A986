
#ifndef __ASM_ARM_ARCH_IO_H
#define __ASM_ARM_ARCH_IO_H

#include <asm/hardware.h>

#define IO_SPACE_LIMIT 0xffffffff

#define __io(a)			((void __iomem *)(PCIO_BASE + (a)))
#define __mem_pci(a)		(a)


#define PCIO_BASE	0

#if defined(CONFIG_ARCH_OMAP1)

#define IO_PHYS		0xFFFB0000
#define IO_OFFSET	0x01000000	/* Virtual IO = 0xfefb0000 */
#define IO_SIZE		0x40000
#define IO_VIRT		(IO_PHYS - IO_OFFSET)
#define IO_ADDRESS(pa)	((pa) - IO_OFFSET)
#define io_p2v(pa)	((pa) - IO_OFFSET)
#define io_v2p(va)	((va) + IO_OFFSET)

#elif defined(CONFIG_ARCH_OMAP2)

/* We map both L3 and L4 on OMAP2 */
#define L3_24XX_PHYS	L3_24XX_BASE	/* 0x68000000 */
#define L3_24XX_VIRT	0xf8000000
#define L3_24XX_SIZE	SZ_1M		/* 44kB of 128MB used, want 1MB sect */
#define L4_24XX_PHYS	L4_24XX_BASE	/* 0x48000000 */
#define L4_24XX_VIRT	0xd8000000
#define L4_24XX_SIZE	SZ_1M		/* 1MB of 128MB used, want 1MB sect */
#define IO_OFFSET	0x90000000
#define IO_ADDRESS(pa)	((pa) + IO_OFFSET)	/* Works for L3 and L4 */
#define io_p2v(pa)	((pa) + IO_OFFSET)	/* Works for L3 and L4 */
#define io_v2p(va)	((va) - IO_OFFSET)	/* Works for L3 and L4 */

#endif

#ifndef __ASSEMBLER__

#define omap_readb(a)		(*(volatile unsigned char  *)IO_ADDRESS(a))
#define omap_readw(a)		(*(volatile unsigned short *)IO_ADDRESS(a))
#define omap_readl(a)		(*(volatile unsigned int   *)IO_ADDRESS(a))

#define omap_writeb(v,a)	(*(volatile unsigned char  *)IO_ADDRESS(a) = (v))
#define omap_writew(v,a)	(*(volatile unsigned short *)IO_ADDRESS(a) = (v))
#define omap_writel(v,a)	(*(volatile unsigned int   *)IO_ADDRESS(a) = (v))

/* 16 bit uses LDRH/STRH, base +/- offset_8 */
typedef struct { volatile u16 offset[256]; } __regbase16;
#define __REGV16(vaddr)		((__regbase16 *)((vaddr)&~0xff)) \
					->offset[((vaddr)&0xff)>>1]
#define __REG16(paddr)          __REGV16(io_p2v(paddr))

/* 8/32 bit uses LDR/STR, base +/- offset_12 */
typedef struct { volatile u8 offset[4096]; } __regbase8;
#define __REGV8(vaddr)		((__regbase8  *)((vaddr)&~4095)) \
					->offset[((vaddr)&4095)>>0]
#define __REG8(paddr)		__REGV8(io_p2v(paddr))

typedef struct { volatile u32 offset[4096]; } __regbase32;
#define __REGV32(vaddr)		((__regbase32 *)((vaddr)&~4095)) \
					->offset[((vaddr)&4095)>>2]
#define __REG32(paddr)		__REGV32(io_p2v(paddr))

extern void omap1_map_common_io(void);
extern void omap1_init_common_hw(void);

extern void omap2_map_common_io(void);
extern void omap2_init_common_hw(void);

#else

#define __REG8(paddr)		io_p2v(paddr)
#define __REG16(paddr)		io_p2v(paddr)
#define __REG32(paddr)		io_p2v(paddr)

#endif

#endif
