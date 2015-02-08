

#ifndef __ASM_ARM_ARCH_IO_H
#define __ASM_ARM_ARCH_IO_H

#include <mach/hardware.h>

#define IO_SPACE_LIMIT 0xffffffff

#define __io(a)		__typesafe_io(a)
#define __mem_pci(a)	(a)


#ifdef __ASSEMBLER__
#define IOMEM(x)		(x)
#else
#define IOMEM(x)		((void __force __iomem *)(x))
#endif

#define OMAP1_IO_OFFSET		0x01000000	/* Virtual IO = 0xfefb0000 */
#define OMAP1_IO_ADDRESS(pa)	IOMEM((pa) - OMAP1_IO_OFFSET)

#define OMAP2_L3_IO_OFFSET	0x90000000
#define OMAP2_L3_IO_ADDRESS(pa)	IOMEM((pa) + OMAP2_L3_IO_OFFSET) /* L3 */


#define OMAP2_L4_IO_OFFSET	0xb2000000
#define OMAP2_L4_IO_ADDRESS(pa)	IOMEM((pa) + OMAP2_L4_IO_OFFSET) /* L4 */

#define OMAP4_L3_IO_OFFSET	0xb4000000
#define OMAP4_L3_IO_ADDRESS(pa)	IOMEM((pa) + OMAP4_L3_IO_OFFSET) /* L3 */

#define OMAP4_L3_PER_IO_OFFSET	0xb1100000
#define OMAP4_L3_PER_IO_ADDRESS(pa)	IOMEM((pa) + OMAP4_L3_PER_IO_OFFSET)

#define OMAP4_GPMC_IO_OFFSET		0xa9000000
#define OMAP4_GPMC_IO_ADDRESS(pa)	IOMEM((pa) + OMAP4_GPMC_IO_OFFSET)

#define OMAP2_EMU_IO_OFFSET		0xaa800000	/* Emulation */
#define OMAP2_EMU_IO_ADDRESS(pa)	IOMEM((pa) + OMAP2_EMU_IO_OFFSET)


#define OMAP1_IO_PHYS		0xFFFB0000
#define OMAP1_IO_SIZE		0x40000
#define OMAP1_IO_VIRT		(OMAP1_IO_PHYS - OMAP1_IO_OFFSET)


/* We map both L3 and L4 on OMAP2 */
#define L3_24XX_PHYS	L3_24XX_BASE	/* 0x68000000 --> 0xf8000000*/
#define L3_24XX_VIRT	(L3_24XX_PHYS + OMAP2_L3_IO_OFFSET)
#define L3_24XX_SIZE	SZ_1M		/* 44kB of 128MB used, want 1MB sect */
#define L4_24XX_PHYS	L4_24XX_BASE	/* 0x48000000 --> 0xfa000000 */
#define L4_24XX_VIRT	(L4_24XX_PHYS + OMAP2_L4_IO_OFFSET)
#define L4_24XX_SIZE	SZ_1M		/* 1MB of 128MB used, want 1MB sect */

#define L4_WK_243X_PHYS		L4_WK_243X_BASE	/* 0x49000000 --> 0xfb000000 */
#define L4_WK_243X_VIRT		(L4_WK_243X_PHYS + OMAP2_L4_IO_OFFSET)
#define L4_WK_243X_SIZE		SZ_1M
#define OMAP243X_GPMC_PHYS	OMAP243X_GPMC_BASE
#define OMAP243X_GPMC_VIRT	(OMAP243X_GPMC_PHYS + OMAP2_L3_IO_OFFSET)
						/* 0x6e000000 --> 0xfe000000 */
#define OMAP243X_GPMC_SIZE	SZ_1M
#define OMAP243X_SDRC_PHYS	OMAP243X_SDRC_BASE
						/* 0x6D000000 --> 0xfd000000 */
#define OMAP243X_SDRC_VIRT	(OMAP243X_SDRC_PHYS + OMAP2_L3_IO_OFFSET)
#define OMAP243X_SDRC_SIZE	SZ_1M
#define OMAP243X_SMS_PHYS	OMAP243X_SMS_BASE
						/* 0x6c000000 --> 0xfc000000 */
#define OMAP243X_SMS_VIRT	(OMAP243X_SMS_PHYS + OMAP2_L3_IO_OFFSET)
#define OMAP243X_SMS_SIZE	SZ_1M

/* 2420 IVA */
#define DSP_MEM_2420_PHYS	OMAP2420_DSP_MEM_BASE
						/* 0x58000000 --> 0xfc100000 */
#define DSP_MEM_2420_VIRT	0xfc100000
#define DSP_MEM_2420_SIZE	0x28000
#define DSP_IPI_2420_PHYS	OMAP2420_DSP_IPI_BASE
						/* 0x59000000 --> 0xfc128000 */
#define DSP_IPI_2420_VIRT	0xfc128000
#define DSP_IPI_2420_SIZE	SZ_4K
#define DSP_MMU_2420_PHYS	OMAP2420_DSP_MMU_BASE
						/* 0x5a000000 --> 0xfc129000 */
#define DSP_MMU_2420_VIRT	0xfc129000
#define DSP_MMU_2420_SIZE	SZ_4K

/* 2430 IVA2.1 - currently unmapped */


/* We map both L3 and L4 on OMAP3 */
#define L3_34XX_PHYS		L3_34XX_BASE	/* 0x68000000 --> 0xf8000000 */
#define L3_34XX_VIRT		(L3_34XX_PHYS + OMAP2_L3_IO_OFFSET)
#define L3_34XX_SIZE		SZ_1M   /* 44kB of 128MB used, want 1MB sect */

#define L4_34XX_PHYS		L4_34XX_BASE	/* 0x48000000 --> 0xfa000000 */
#define L4_34XX_VIRT		(L4_34XX_PHYS + OMAP2_L4_IO_OFFSET)
#define L4_34XX_SIZE		SZ_4M   /* 1MB of 128MB used, want 1MB sect */


#define L4_PER_34XX_PHYS	L4_PER_34XX_BASE
						/* 0x49000000 --> 0xfb000000 */
#define L4_PER_34XX_VIRT	(L4_PER_34XX_PHYS + OMAP2_L4_IO_OFFSET)
#define L4_PER_34XX_SIZE	SZ_1M

#define L4_EMU_34XX_PHYS	L4_EMU_34XX_BASE
						/* 0x54000000 --> 0xfe800000 */
#define L4_EMU_34XX_VIRT	(L4_EMU_34XX_PHYS + OMAP2_EMU_IO_OFFSET)
#define L4_EMU_34XX_SIZE	SZ_8M

#define OMAP34XX_GPMC_PHYS	OMAP34XX_GPMC_BASE
						/* 0x6e000000 --> 0xfe000000 */
#define OMAP34XX_GPMC_VIRT	(OMAP34XX_GPMC_PHYS + OMAP2_L3_IO_OFFSET)
#define OMAP34XX_GPMC_SIZE	SZ_1M

#define OMAP343X_SMS_PHYS	OMAP343X_SMS_BASE
						/* 0x6c000000 --> 0xfc000000 */
#define OMAP343X_SMS_VIRT	(OMAP343X_SMS_PHYS + OMAP2_L3_IO_OFFSET)
#define OMAP343X_SMS_SIZE	SZ_1M

#define OMAP343X_SDRC_PHYS	OMAP343X_SDRC_BASE
						/* 0x6D000000 --> 0xfd000000 */
#define OMAP343X_SDRC_VIRT	(OMAP343X_SDRC_PHYS + OMAP2_L3_IO_OFFSET)
#define OMAP343X_SDRC_SIZE	SZ_1M

/* 3430 IVA - currently unmapped */


/* We map both L3 and L4 on OMAP4 */
#define L3_44XX_PHYS		L3_44XX_BASE	/* 0x44000000 --> 0xf8000000 */
#define L3_44XX_VIRT		(L3_44XX_PHYS + OMAP4_L3_IO_OFFSET)
#define L3_44XX_SIZE		SZ_1M

#define L4_44XX_PHYS		L4_44XX_BASE	/* 0x4a000000 --> 0xfc000000 */
#define L4_44XX_VIRT		(L4_44XX_PHYS + OMAP2_L4_IO_OFFSET)
#define L4_44XX_SIZE		SZ_4M

#define L4_PER_44XX_PHYS	L4_PER_44XX_BASE
						/* 0x48000000 --> 0xfa000000 */
#define L4_PER_44XX_VIRT	(L4_PER_44XX_PHYS + OMAP2_L4_IO_OFFSET)
#define L4_PER_44XX_SIZE	SZ_4M

#define L4_ABE_44XX_PHYS	L4_ABE_44XX_BASE
						/* 0x49000000 --> 0xfb000000 */
#define L4_ABE_44XX_VIRT	(L4_ABE_44XX_PHYS + OMAP2_L4_IO_OFFSET)
#define L4_ABE_44XX_SIZE	SZ_1M

#define L4_EMU_44XX_PHYS	L4_EMU_44XX_BASE
						/* 0x54000000 --> 0xfe800000 */
#define L4_EMU_44XX_VIRT	(L4_EMU_44XX_PHYS + OMAP2_EMU_IO_OFFSET)
#define L4_EMU_44XX_SIZE	SZ_8M

#define OMAP44XX_GPMC_PHYS	OMAP44XX_GPMC_BASE
						/* 0x50000000 --> 0xf9000000 */
#define OMAP44XX_GPMC_VIRT	(OMAP44XX_GPMC_PHYS + OMAP4_GPMC_IO_OFFSET)
#define OMAP44XX_GPMC_SIZE	SZ_1M


#define OMAP44XX_EMIF1_PHYS	OMAP44XX_EMIF1_BASE
						/* 0x4c000000 --> 0xfd100000 */
#define OMAP44XX_EMIF1_VIRT	(OMAP44XX_EMIF1_PHYS + OMAP4_L3_PER_IO_OFFSET)
#define OMAP44XX_EMIF1_SIZE	SZ_1M

#define OMAP44XX_EMIF2_PHYS	OMAP44XX_EMIF2_BASE
						/* 0x4d000000 --> 0xfd200000 */
#define OMAP44XX_EMIF2_VIRT	(OMAP44XX_EMIF2_PHYS + OMAP4_L3_PER_IO_OFFSET)
#define OMAP44XX_EMIF2_SIZE	SZ_1M

#define OMAP44XX_DMM_PHYS	OMAP44XX_DMM_BASE
						/* 0x4e000000 --> 0xfd300000 */
#define OMAP44XX_DMM_VIRT	(OMAP44XX_DMM_PHYS + OMAP4_L3_PER_IO_OFFSET)
#define OMAP44XX_DMM_SIZE	SZ_1M

#ifndef __ASSEMBLER__


extern u8 omap_readb(u32 pa);
extern u16 omap_readw(u32 pa);
extern u32 omap_readl(u32 pa);
extern void omap_writeb(u8 v, u32 pa);
extern void omap_writew(u16 v, u32 pa);
extern void omap_writel(u32 v, u32 pa);

struct omap_sdrc_params;

extern void omap1_map_common_io(void);
extern void omap1_init_common_hw(void);

#ifdef CONFIG_ARCH_OMAP2420
extern void omap242x_map_common_io(void);
#else
static inline void omap242x_map_common_io(void)
{
}
#endif

#ifdef CONFIG_ARCH_OMAP2430
extern void omap243x_map_common_io(void);
#else
static inline void omap243x_map_common_io(void)
{
}
#endif

#ifdef CONFIG_ARCH_OMAP3
extern void omap34xx_map_common_io(void);
#else
static inline void omap34xx_map_common_io(void)
{
}
#endif

#ifdef CONFIG_ARCH_OMAP4
extern void omap44xx_map_common_io(void);
#else
static inline void omap44xx_map_common_io(void)
{
}
#endif

extern void omap2_init_common_hw(struct omap_sdrc_params *sdrc_cs0,
				 struct omap_sdrc_params *sdrc_cs1);

#define __arch_ioremap(p,s,t)	omap_ioremap(p,s,t)
#define __arch_iounmap(v)	omap_iounmap(v)

void __iomem *omap_ioremap(unsigned long phys, size_t size, unsigned int type);
void omap_iounmap(volatile void __iomem *addr);

#endif

#endif
