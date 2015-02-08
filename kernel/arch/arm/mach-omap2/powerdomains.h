


#ifndef ARCH_ARM_MACH_OMAP2_POWERDOMAINS
#define ARCH_ARM_MACH_OMAP2_POWERDOMAINS



#include <plat/powerdomain.h>

#include "prcm-common.h"
#include "prm.h"
#include "cm.h"
#include "powerdomains24xx.h"
#include "powerdomains34xx.h"
#include "powerdomains44xx.h"

/* OMAP2/3-common powerdomains */

#if defined(CONFIG_ARCH_OMAP2) || defined(CONFIG_ARCH_OMAP3)

static struct powerdomain gfx_omap2_pwrdm = {
	.name		  = "gfx_pwrdm",
	.prcm_offs	  = GFX_MOD,
	.omap_chip	  = OMAP_CHIP_INIT(CHIP_IS_OMAP24XX |
					   CHIP_IS_OMAP3430ES1),
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

static struct powerdomain wkup_omap2_pwrdm = {
	.name		= "wkup_pwrdm",
	.prcm_offs	= WKUP_MOD,
	.omap_chip	= OMAP_CHIP_INIT(CHIP_IS_OMAP24XX | CHIP_IS_OMAP3430),
};

#endif


/* As powerdomains are added or removed above, this list must also be changed */
static struct powerdomain *powerdomains_omap[] __initdata = {

#if defined(CONFIG_ARCH_OMAP2) || defined(CONFIG_ARCH_OMAP3)
	&wkup_omap2_pwrdm,
	&gfx_omap2_pwrdm,
#endif

#ifdef CONFIG_ARCH_OMAP2
	&dsp_pwrdm,
	&mpu_24xx_pwrdm,
	&core_24xx_pwrdm,
#endif

#ifdef CONFIG_ARCH_OMAP2430
	&mdm_pwrdm,
#endif

#ifdef CONFIG_ARCH_OMAP3
	&iva2_pwrdm,
	&mpu_3xxx_pwrdm,
	&neon_pwrdm,
	&core_3xxx_pre_es3_1_pwrdm,
	&core_3xxx_es3_1_pwrdm,
	&cam_pwrdm,
	&dss_pwrdm,
	&per_pwrdm,
	&emu_pwrdm,
	&sgx_pwrdm,
	&usbhost_pwrdm,
	&dpll1_pwrdm,
	&dpll2_pwrdm,
	&dpll3_pwrdm,
	&dpll4_pwrdm,
	&dpll5_pwrdm,
#endif

#ifdef CONFIG_ARCH_OMAP4
	&core_44xx_pwrdm,
	&gfx_44xx_pwrdm,
	&abe_44xx_pwrdm,
	&dss_44xx_pwrdm,
	&tesla_44xx_pwrdm,
	&wkup_44xx_pwrdm,
	&cpu0_44xx_pwrdm,
	&cpu1_44xx_pwrdm,
	&emu_44xx_pwrdm,
	&mpu_44xx_pwrdm,
	&ivahd_44xx_pwrdm,
	&cam_44xx_pwrdm,
	&l3init_44xx_pwrdm,
	&l4per_44xx_pwrdm,
	&always_on_core_44xx_pwrdm,
	&cefuse_44xx_pwrdm,
#endif
	NULL
};


#endif
