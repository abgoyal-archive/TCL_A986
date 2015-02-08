

#include <plat/omap_hwmod.h>

#include "omap_hwmod_common_data.h"

struct omap_hwmod_sysc_fields omap_hwmod_sysc_type1 = {
	.midle_shift	= SYSC_TYPE1_MIDLEMODE_SHIFT,
	.clkact_shift	= SYSC_TYPE1_CLOCKACTIVITY_SHIFT,
	.sidle_shift	= SYSC_TYPE1_SIDLEMODE_SHIFT,
	.enwkup_shift	= SYSC_TYPE1_ENAWAKEUP_SHIFT,
	.srst_shift	= SYSC_TYPE1_SOFTRESET_SHIFT,
	.autoidle_shift	= SYSC_TYPE1_AUTOIDLE_SHIFT,
};

struct omap_hwmod_sysc_fields omap_hwmod_sysc_type2 = {
	.midle_shift	= SYSC_TYPE2_MIDLEMODE_SHIFT,
	.sidle_shift	= SYSC_TYPE2_SIDLEMODE_SHIFT,
	.srst_shift	= SYSC_TYPE2_SOFTRESET_SHIFT,
};



struct omap_hwmod_class l3_hwmod_class = {
	.name = "l3"
};

struct omap_hwmod_class l4_hwmod_class = {
	.name = "l4"
};

struct omap_hwmod_class mpu_hwmod_class = {
	.name = "mpu"
};

