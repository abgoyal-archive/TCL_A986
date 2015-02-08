

#ifndef __ARCH_ARM_MACH_OMAP2_SDRAM_HYNIX_H8MBX00U0MER0EM
#define __ARCH_ARM_MACH_OMAP2_SDRAM_HYNIX_H8MBX00U0MER0EM

#include <plat/sdrc.h>

/* Hynix H8MBX00U0MER-0EM */
static struct omap_sdrc_params h8mbx00u0mer0em_sdrc_params[] = {
	[0] = {
		.rate        = 200000000,
		.actim_ctrla = 0xa2e1b4c6,
		.actim_ctrlb = 0x0002131c,
		.rfr_ctrl    = 0x0005e601,
		.mr          = 0x00000032,
	},
	[1] = {
		.rate        = 166000000,
		.actim_ctrla = 0x629db4c6,
		.actim_ctrlb = 0x00012214,
		.rfr_ctrl    = 0x0004dc01,
		.mr          = 0x00000032,
	},
	[2] = {
		.rate        = 100000000,
		.actim_ctrla = 0x51912284,
		.actim_ctrlb = 0x0002120e,
		.rfr_ctrl    = 0x0002d101,
		.mr          = 0x00000022,
	},
	[3] = {
		.rate        = 83000000,
		.actim_ctrla = 0x31512283,
		.actim_ctrlb = 0x0001220a,
		.rfr_ctrl    = 0x00025501,
		.mr          = 0x00000022,
	},
	[4] = {
		.rate        = 0
	},
};

#endif
