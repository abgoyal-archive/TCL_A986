

#include <linux/mmc/sh_mmcif.h>
#include <mach/romimage.h>

#define MMCIF_BASE      (void __iomem *)0xa4ca0000

#define MSTPCR2		0xa4150038
#define PTWCR		0xa4050146
#define PTXCR		0xa4050148
#define PSELA		0xa405014e
#define PSELE		0xa4050156
#define HIZCRC		0xa405015c
#define DRVCRA		0xa405018a

enum { MMCIF_PROGRESS_ENTER, MMCIF_PROGRESS_INIT,
       MMCIF_PROGRESS_LOAD, MMCIF_PROGRESS_DONE };

asmlinkage void mmcif_loader(unsigned char *buf, unsigned long no_bytes)
{
	mmcif_update_progress(MMCIF_PROGRESS_ENTER);

	/* enable clock to the MMCIF hardware block */
	__raw_writel(__raw_readl(MSTPCR2) & ~0x20000000, MSTPCR2);

	/* setup pins D7-D0 */
	__raw_writew(0x0000, PTWCR);

	/* setup pins MMC_CLK, MMC_CMD */
	__raw_writew(__raw_readw(PTXCR) & ~0x000f, PTXCR);

	/* select D3-D0 pin function */
	__raw_writew(__raw_readw(PSELA) & ~0x2000, PSELA);

	/* select D7-D4 pin function */
	__raw_writew(__raw_readw(PSELE) & ~0x3000, PSELE);

	/* disable Hi-Z for the MMC pins */
	__raw_writew(__raw_readw(HIZCRC) & ~0x0620, HIZCRC);

	/* high drive capability for MMC pins */
	__raw_writew(__raw_readw(DRVCRA) | 0x3000, DRVCRA);

	mmcif_update_progress(MMCIF_PROGRESS_INIT);

	/* setup MMCIF hardware */
	sh_mmcif_boot_init(MMCIF_BASE);

	mmcif_update_progress(MMCIF_PROGRESS_LOAD);

	/* load kernel via MMCIF interface */
	sh_mmcif_boot_slurp(MMCIF_BASE, buf, no_bytes);

	/* disable clock to the MMCIF hardware block */
	__raw_writel(__raw_readl(MSTPCR2) | 0x20000000, MSTPCR2);

	mmcif_update_progress(MMCIF_PROGRESS_DONE);
}
