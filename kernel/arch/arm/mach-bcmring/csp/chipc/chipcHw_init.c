

/****************************************************************************/
/****************************************************************************/

/* ---- Include Files ---------------------------------------------------- */

#include <csp/errno.h>
#include <csp/stdint.h>
#include <csp/module.h>

#include <mach/csp/chipcHw_def.h>
#include <mach/csp/chipcHw_inline.h>

#include <csp/reg.h>
#include <csp/delay.h>
/* ---- Private Constants and Types --------------------------------------- */


/* ---- Private Variables ------------------------------------------------- */
/****************************************************************************/
/****************************************************************************/
void chipcHw_pll2Enable(uint32_t vcoFreqHz)
{
	uint32_t pllPreDivider2 = 0;

	{
		REG_LOCAL_IRQ_SAVE;
		pChipcHw->PLLConfig2 =
		    chipcHw_REG_PLL_CONFIG_D_RESET |
		    chipcHw_REG_PLL_CONFIG_A_RESET;

		pllPreDivider2 = chipcHw_REG_PLL_PREDIVIDER_POWER_DOWN |
		    chipcHw_REG_PLL_PREDIVIDER_NDIV_MODE_INTEGER |
		    (chipcHw_REG_PLL_PREDIVIDER_NDIV_i(vcoFreqHz) <<
		     chipcHw_REG_PLL_PREDIVIDER_NDIV_SHIFT) |
		    (chipcHw_REG_PLL_PREDIVIDER_P1 <<
		     chipcHw_REG_PLL_PREDIVIDER_P1_SHIFT) |
		    (chipcHw_REG_PLL_PREDIVIDER_P2 <<
		     chipcHw_REG_PLL_PREDIVIDER_P2_SHIFT);

		/* Enable CHIPC registers to control the PLL */
		pChipcHw->PLLStatus |= chipcHw_REG_PLL_STATUS_CONTROL_ENABLE;

		/* Set pre divider to get desired VCO frequency */
		pChipcHw->PLLPreDivider2 = pllPreDivider2;
		/* Set NDIV Frac */
		pChipcHw->PLLDivider2 = chipcHw_REG_PLL_DIVIDER_NDIV_f;

		/* This has to be removed once the default values are fixed for PLL2. */
		pChipcHw->PLLControl12 = 0x38000700;
		pChipcHw->PLLControl22 = 0x00000015;

		/* Reset PLL2 */
		if (vcoFreqHz > chipcHw_REG_PLL_CONFIG_VCO_SPLIT_FREQ) {
			pChipcHw->PLLConfig2 = chipcHw_REG_PLL_CONFIG_D_RESET |
			    chipcHw_REG_PLL_CONFIG_A_RESET |
			    chipcHw_REG_PLL_CONFIG_VCO_1601_3200 |
			    chipcHw_REG_PLL_CONFIG_POWER_DOWN;
		} else {
			pChipcHw->PLLConfig2 = chipcHw_REG_PLL_CONFIG_D_RESET |
			    chipcHw_REG_PLL_CONFIG_A_RESET |
			    chipcHw_REG_PLL_CONFIG_VCO_800_1600 |
			    chipcHw_REG_PLL_CONFIG_POWER_DOWN;
		}
		REG_LOCAL_IRQ_RESTORE;
	}

	/* Insert certain amount of delay before deasserting ARESET. */
	udelay(1);

	{
		REG_LOCAL_IRQ_SAVE;
		/* Remove analog reset and Power on the PLL */
		pChipcHw->PLLConfig2 &=
		    ~(chipcHw_REG_PLL_CONFIG_A_RESET |
		      chipcHw_REG_PLL_CONFIG_POWER_DOWN);

		REG_LOCAL_IRQ_RESTORE;

	}

	/* Wait until PLL is locked */
	while (!(pChipcHw->PLLStatus2 & chipcHw_REG_PLL_STATUS_LOCKED))
		;

	{
		REG_LOCAL_IRQ_SAVE;
		/* Remove digital reset */
		pChipcHw->PLLConfig2 &= ~chipcHw_REG_PLL_CONFIG_D_RESET;

		REG_LOCAL_IRQ_RESTORE;
	}
}

EXPORT_SYMBOL(chipcHw_pll2Enable);

/****************************************************************************/
/****************************************************************************/
void chipcHw_pll1Enable(uint32_t vcoFreqHz, chipcHw_SPREAD_SPECTRUM_e ssSupport)
{
	uint32_t pllPreDivider = 0;

	{
		REG_LOCAL_IRQ_SAVE;

		pChipcHw->PLLConfig =
		    chipcHw_REG_PLL_CONFIG_D_RESET |
		    chipcHw_REG_PLL_CONFIG_A_RESET;
		/* Setting VCO frequency */
		if (ssSupport == chipcHw_SPREAD_SPECTRUM_ALLOW) {
			pllPreDivider =
			    chipcHw_REG_PLL_PREDIVIDER_NDIV_MODE_MASH_1_8 |
			    ((chipcHw_REG_PLL_PREDIVIDER_NDIV_i(vcoFreqHz) -
			      1) << chipcHw_REG_PLL_PREDIVIDER_NDIV_SHIFT) |
			    (chipcHw_REG_PLL_PREDIVIDER_P1 <<
			     chipcHw_REG_PLL_PREDIVIDER_P1_SHIFT) |
			    (chipcHw_REG_PLL_PREDIVIDER_P2 <<
			     chipcHw_REG_PLL_PREDIVIDER_P2_SHIFT);
		} else {
			pllPreDivider = chipcHw_REG_PLL_PREDIVIDER_POWER_DOWN |
			    chipcHw_REG_PLL_PREDIVIDER_NDIV_MODE_INTEGER |
			    (chipcHw_REG_PLL_PREDIVIDER_NDIV_i(vcoFreqHz) <<
			     chipcHw_REG_PLL_PREDIVIDER_NDIV_SHIFT) |
			    (chipcHw_REG_PLL_PREDIVIDER_P1 <<
			     chipcHw_REG_PLL_PREDIVIDER_P1_SHIFT) |
			    (chipcHw_REG_PLL_PREDIVIDER_P2 <<
			     chipcHw_REG_PLL_PREDIVIDER_P2_SHIFT);
		}

		/* Enable CHIPC registers to control the PLL */
		pChipcHw->PLLStatus |= chipcHw_REG_PLL_STATUS_CONTROL_ENABLE;

		/* Set pre divider to get desired VCO frequency */
		pChipcHw->PLLPreDivider = pllPreDivider;
		/* Set NDIV Frac */
		if (ssSupport == chipcHw_SPREAD_SPECTRUM_ALLOW) {
			pChipcHw->PLLDivider = chipcHw_REG_PLL_DIVIDER_M1DIV |
			    chipcHw_REG_PLL_DIVIDER_NDIV_f_SS;
		} else {
			pChipcHw->PLLDivider = chipcHw_REG_PLL_DIVIDER_M1DIV |
			    chipcHw_REG_PLL_DIVIDER_NDIV_f;
		}

		/* Reset PLL1 */
		if (vcoFreqHz > chipcHw_REG_PLL_CONFIG_VCO_SPLIT_FREQ) {
			pChipcHw->PLLConfig = chipcHw_REG_PLL_CONFIG_D_RESET |
			    chipcHw_REG_PLL_CONFIG_A_RESET |
			    chipcHw_REG_PLL_CONFIG_VCO_1601_3200 |
			    chipcHw_REG_PLL_CONFIG_POWER_DOWN;
		} else {
			pChipcHw->PLLConfig = chipcHw_REG_PLL_CONFIG_D_RESET |
			    chipcHw_REG_PLL_CONFIG_A_RESET |
			    chipcHw_REG_PLL_CONFIG_VCO_800_1600 |
			    chipcHw_REG_PLL_CONFIG_POWER_DOWN;
		}

		REG_LOCAL_IRQ_RESTORE;

		/* Insert certain amount of delay before deasserting ARESET. */
		udelay(1);

		{
			REG_LOCAL_IRQ_SAVE;
			/* Remove analog reset and Power on the PLL */
			pChipcHw->PLLConfig &=
			    ~(chipcHw_REG_PLL_CONFIG_A_RESET |
			      chipcHw_REG_PLL_CONFIG_POWER_DOWN);
			REG_LOCAL_IRQ_RESTORE;
		}

		/* Wait until PLL is locked */
		while (!(pChipcHw->PLLStatus & chipcHw_REG_PLL_STATUS_LOCKED)
		       || !(pChipcHw->
			    PLLStatus2 & chipcHw_REG_PLL_STATUS_LOCKED))
			;

		/* Remove digital reset */
		{
			REG_LOCAL_IRQ_SAVE;
			pChipcHw->PLLConfig &= ~chipcHw_REG_PLL_CONFIG_D_RESET;
			REG_LOCAL_IRQ_RESTORE;
		}
	}
}

EXPORT_SYMBOL(chipcHw_pll1Enable);

/****************************************************************************/
/****************************************************************************/

void chipcHw_Init(chipcHw_INIT_PARAM_t *initParam	/*  [ IN ] Misc chip initialization parameter */
    ) {
#if !(defined(__KERNEL__) && !defined(STANDALONE))
	delay_init();
#endif

	/* Do not program PLL, when warm reset */
	if (!(chipcHw_getStickyBits() & chipcHw_REG_STICKY_CHIP_WARM_RESET)) {
		chipcHw_pll1Enable(initParam->pllVcoFreqHz,
				   initParam->ssSupport);
		chipcHw_pll2Enable(initParam->pll2VcoFreqHz);
	} else {
		/* Clear sticky bits */
		chipcHw_clearStickyBits(chipcHw_REG_STICKY_CHIP_WARM_RESET);
	}
	/* Clear sticky bits */
	chipcHw_clearStickyBits(chipcHw_REG_STICKY_CHIP_SOFT_RESET);

	/* Before configuring the ARM clock, atleast we need to make sure BUS clock maintains the proper ratio with ARM clock */
	pChipcHw->ACLKClock =
	    (pChipcHw->
	     ACLKClock & ~chipcHw_REG_ACLKClock_CLK_DIV_MASK) | (initParam->
								 armBusRatio &
								 chipcHw_REG_ACLKClock_CLK_DIV_MASK);

	/* Set various core component frequencies. The order in which this is done is important for some. */
	/* The RTBUS (DDR PHY) is derived from the BUS, and the BUS from the ARM, and VPM needs to know BUS */
	/* frequency to find its ratio with the BUS.  Hence we must set the ARM first, followed by the BUS,  */
	/* then VPM and RTBUS. */

	chipcHw_setClockFrequency(chipcHw_CLOCK_ARM,
				  initParam->busClockFreqHz *
				  initParam->armBusRatio);
	chipcHw_setClockFrequency(chipcHw_CLOCK_BUS, initParam->busClockFreqHz);
	chipcHw_setClockFrequency(chipcHw_CLOCK_VPM,
				  initParam->busClockFreqHz *
				  initParam->vpmBusRatio);
	chipcHw_setClockFrequency(chipcHw_CLOCK_DDR,
				  initParam->busClockFreqHz *
				  initParam->ddrBusRatio);
	chipcHw_setClockFrequency(chipcHw_CLOCK_RTBUS,
				  initParam->busClockFreqHz / 2);
}
