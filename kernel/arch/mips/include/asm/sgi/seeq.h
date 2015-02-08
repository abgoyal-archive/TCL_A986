
#ifndef __ASM_SGI_SEEQ_H
#define __ASM_SGI_SEEQ_H

#include <linux/if_ether.h>

#include <asm/sgi/hpc3.h>

struct sgiseeq_platform_data {
	struct hpc3_regs *hpc;
	unsigned int irq;
	unsigned char mac[ETH_ALEN];
};

#endif /* __ASM_SGI_SEEQ_H */
