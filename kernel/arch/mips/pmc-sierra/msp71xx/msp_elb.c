

#include <linux/kernel.h>
#include <linux/init.h>
#include <msp_regs.h>

static int __init msp_elb_setup(void)
{
#if defined(CONFIG_PMC_MSP7120_GW) \
 || defined(CONFIG_PMC_MSP7120_EVAL)
	/*
	 * Force all CNFG to be identical and equal to CS0,
	 * according to OPS doc
	 */
	*CS1_CNFG_REG = *CS2_CNFG_REG = *CS3_CNFG_REG = *CS0_CNFG_REG;
#endif
	return 0;
}

subsys_initcall(msp_elb_setup);
