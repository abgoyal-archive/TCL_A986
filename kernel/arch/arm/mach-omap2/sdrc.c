
#undef DEBUG

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/io.h>

#include <plat/common.h>
#include <plat/clock.h>
#include <plat/sram.h>

#include "prm.h"

#include <plat/sdrc.h>
#include "sdrc.h"

static struct omap_sdrc_params *sdrc_init_params_cs0, *sdrc_init_params_cs1;

void __iomem *omap2_sdrc_base;
void __iomem *omap2_sms_base;

struct omap2_sms_regs {
	u32	sms_sysconfig;
};

static struct omap2_sms_regs sms_context;

/* SDRC_POWER register bits */
#define SDRC_POWER_EXTCLKDIS_SHIFT		3
#define SDRC_POWER_PWDENA_SHIFT			2
#define SDRC_POWER_PAGEPOLICY_SHIFT		0

void omap2_sms_save_context(void)
{
	sms_context.sms_sysconfig = sms_read_reg(SMS_SYSCONFIG);
}

void omap2_sms_restore_context(void)
{
	sms_write_reg(sms_context.sms_sysconfig, SMS_SYSCONFIG);
}

int omap2_sdrc_get_params(unsigned long r,
			  struct omap_sdrc_params **sdrc_cs0,
			  struct omap_sdrc_params **sdrc_cs1)
{
	struct omap_sdrc_params *sp0, *sp1;

	if (!sdrc_init_params_cs0)
		return -1;

	sp0 = sdrc_init_params_cs0;
	sp1 = sdrc_init_params_cs1;

	while (sp0->rate && sp0->rate != r) {
		sp0++;
		if (sdrc_init_params_cs1)
			sp1++;
	}

	if (!sp0->rate)
		return -1;

	*sdrc_cs0 = sp0;
	*sdrc_cs1 = sp1;
	return 0;
}


void __init omap2_set_globals_sdrc(struct omap_globals *omap2_globals)
{
	/* Static mapping, never released */
	if (omap2_globals->sdrc) {
		omap2_sdrc_base = ioremap(omap2_globals->sdrc, SZ_64K);
		WARN_ON(!omap2_sdrc_base);
	}
	if (omap2_globals->sms) {
		omap2_sms_base = ioremap(omap2_globals->sms, SZ_64K);
		WARN_ON(!omap2_sms_base);
	}
}

void __init omap2_sdrc_init(struct omap_sdrc_params *sdrc_cs0,
			    struct omap_sdrc_params *sdrc_cs1)
{
	u32 l;

	l = sms_read_reg(SMS_SYSCONFIG);
	l &= ~(0x3 << 3);
	l |= (0x2 << 3);
	sms_write_reg(l, SMS_SYSCONFIG);

	l = sdrc_read_reg(SDRC_SYSCONFIG);
	l &= ~(0x3 << 3);
	l |= (0x2 << 3);
	sdrc_write_reg(l, SDRC_SYSCONFIG);

	sdrc_init_params_cs0 = sdrc_cs0;
	sdrc_init_params_cs1 = sdrc_cs1;

	/* XXX Enable SRFRONIDLEREQ here also? */
	/*
	 * PWDENA should not be set due to 34xx erratum 1.150 - PWDENA
	 * can cause random memory corruption
	 */
	l = (1 << SDRC_POWER_EXTCLKDIS_SHIFT) |
		(1 << SDRC_POWER_PAGEPOLICY_SHIFT);
	sdrc_write_reg(l, SDRC_POWER);
	omap2_sms_save_context();
}

void omap2_sms_write_rot_control(u32 val, unsigned ctx)
{
	sms_write_reg(val, SMS_ROT_CONTROL(ctx));
}

void omap2_sms_write_rot_size(u32 val, unsigned ctx)
{
	sms_write_reg(val, SMS_ROT_SIZE(ctx));
}

void omap2_sms_write_rot_physical_ba(u32 val, unsigned ctx)
{
	sms_write_reg(val, SMS_ROT_PHYSICAL_BA(ctx));
}

