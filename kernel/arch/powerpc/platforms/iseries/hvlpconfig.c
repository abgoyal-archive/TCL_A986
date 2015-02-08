

#include <linux/module.h>
#include <asm/iseries/hv_lp_config.h>
#include "it_lp_naca.h"

HvLpIndex HvLpConfig_getLpIndex_outline(void)
{
	return HvLpConfig_getLpIndex();
}
EXPORT_SYMBOL(HvLpConfig_getLpIndex_outline);

HvLpIndex HvLpConfig_getLpIndex(void)
{
	return itLpNaca.xLpIndex;
}
EXPORT_SYMBOL(HvLpConfig_getLpIndex);

HvLpIndex HvLpConfig_getPrimaryLpIndex(void)
{
	return itLpNaca.xPrimaryLpIndex;
}
EXPORT_SYMBOL_GPL(HvLpConfig_getPrimaryLpIndex);
