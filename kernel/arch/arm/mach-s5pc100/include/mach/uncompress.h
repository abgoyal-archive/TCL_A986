

#ifndef __ASM_ARCH_UNCOMPRESS_H
#define __ASM_ARCH_UNCOMPRESS_H

#include <mach/map.h>
#include <plat/uncompress.h>

static void arch_detect_cpu(void)
{
	/* we do not need to do any cpu detection here at the moment. */
	fifo_mask = S3C2440_UFSTAT_TXMASK;
	fifo_max = 63 << S3C2440_UFSTAT_TXSHIFT;
}

#endif /* __ASM_ARCH_UNCOMPRESS_H */
