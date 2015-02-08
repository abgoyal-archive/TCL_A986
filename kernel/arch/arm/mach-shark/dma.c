

#include <linux/init.h>

#include <asm/dma.h>
#include <asm/mach/dma.h>

static int __init shark_dma_init(void)
{
#ifdef CONFIG_ISA_DMA
	isa_init_dma();
#endif
	return 0;
}
core_initcall(shark_dma_init);
