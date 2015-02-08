

#ifndef ARCH_ARM_MACH_OMAP2_POWERDOMAINS24XX
#define ARCH_ARM_MACH_OMAP2_POWERDOMAINS24XX


#include <plat/powerdomain.h>

#include "prcm-common.h"
#include "prm.h"
#include "prm-regbits-24xx.h"
#include "cm.h"
#include "cm-regbits-24xx.h"

/* 24XX powerdomains and dependencies */

#ifdef CONFIG_ARCH_OMAP2

/* Powerdomains */

static struct powerdomain dsp_pwrdm = {
	.name		  = "dsp_pwrdm",
	.prcm_offs	  = OMAP24XX_DSP_MOD,
	.omap_chip	  = OMAP_CHIP_INIT(CHIP_IS_OMAP24XX),
	.pwrsts		  = PWRSTS_OFF_RET_ON,
	.pwrsts_logic_ret = PWRDM_POWER_RET,
	.banks		  = 1,
	.pwrsts_mem_ret	  = {
		[0] = PWRDM_POWER_RET,
	},
	.pwrsts_mem_on	  = {
		[0] = PWRDM_POWER_ON,
	},
};

static struct powerdomain mpu_24xx_pwrdm = {
	.name		  = "mpu_pwrdm",
	.prcm_offs	  = MPU_MOD,
	.omap_chip	  = OMAP_CHIP_INIT(CHIP_IS_OMAP24XX),
	.pwrsts		  = PWRSTS_OFF_RET_ON,
	.pwrsts_logic_ret = PWRSTS_OFF_RET,
	.banks		  = 1,
	.pwrsts_mem_ret	  = {
		[0] = PWRDM_POWER_RET,
	},
	.pwrsts_mem_on	  = {
		[0] = PWRDM_POWER_ON,
	},
};

static struct powerdomain core_24xx_pwrdm = {
	.name		  = "core_pwrdm",
	.prcm_offs	  = CORE_MOD,
	.omap_chip	  = OMAP_CHIP_INIT(CHIP_IS_OMAP24XX),
	.pwrsts		  = PWRSTS_OFF_RET_ON,
	.banks		  = 3,
	.pwrsts_mem_ret	  = {
		[0] = PWRSTS_OFF_RET,	 /* MEM1RETSTATE */
		[1] = PWRSTS_OFF_RET,	 /* MEM2RETSTATE */
		[2] = PWRSTS_OFF_RET,	 /* MEM3RETSTATE */
	},
	.pwrsts_mem_on	  = {
		[0] = PWRSTS_OFF_RET_ON, /* MEM1ONSTATE */
		[1] = PWRSTS_OFF_RET_ON, /* MEM2ONSTATE */
		[2] = PWRSTS_OFF_RET_ON, /* MEM3ONSTATE */
	},
};

#endif	   /* CONFIG_ARCH_OMAP2 */




#ifdef CONFIG_ARCH_OMAP2430

/* XXX 2430 KILLDOMAINWKUP bit?  No current users apparently */

static struct powerdomain mdm_pwrdm = {
	.name		  = "mdm_pwrdm",
	.prcm_offs	  = OMAP2430_MDM_MOD,
	.omap_chip	  = OMAP_CHIP_INIT(CHIP_IS_OMAP2430),
	.pwrsts		  = PWRSTS_OFF_RET_ON,
	.pwrsts_logic_ret = PWRDM_POWER_RET,
	.banks		  = 1,
	.pwrsts_mem_ret	  = {
		[0] = PWRDM_POWER_RET, /* MEMRETSTATE */
	},
	.pwrsts_mem_on	  = {
		[0] = PWRDM_POWER_ON,  /* MEMONSTATE */
	},
};

#endif     /* CONFIG_ARCH_OMAP2430 */


#endif
