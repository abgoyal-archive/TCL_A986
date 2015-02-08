
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/smp.h>
#include <linux/spinlock.h>
#include <linux/io.h>

#include <mach/hardware.h>
#include <mach/platform.h>
#include <asm/leds.h>
#include <asm/system.h>
#include <asm/mach-types.h>
#include <mach/cm.h>

static int saved_leds;

static void integrator_leds_event(led_event_t ledevt)
{
	unsigned long flags;
	const unsigned int dbg_base = IO_ADDRESS(INTEGRATOR_DBG_BASE);
	unsigned int update_alpha_leds;

	// yup, change the LEDs
	local_irq_save(flags);
	update_alpha_leds = 0;

	switch(ledevt) {
	case led_idle_start:
		cm_control(CM_CTRL_LED, 0);
		break;

	case led_idle_end:
		cm_control(CM_CTRL_LED, CM_CTRL_LED);
		break;

	case led_timer:
		saved_leds ^= GREEN_LED;
		update_alpha_leds = 1;
		break;

	case led_red_on:
		saved_leds |= RED_LED;
		update_alpha_leds = 1;
		break;

	case led_red_off:
		saved_leds &= ~RED_LED;
		update_alpha_leds = 1;
		break;

	default:
		break;
	}

	if (update_alpha_leds) {
		while (__raw_readl(dbg_base + INTEGRATOR_DBG_ALPHA_OFFSET) & 1);
		__raw_writel(saved_leds, dbg_base + INTEGRATOR_DBG_LEDS_OFFSET);
	}
	local_irq_restore(flags);
}

static int __init leds_init(void)
{
	if (machine_is_integrator() || machine_is_cintegrator())
		leds_event = integrator_leds_event;

	return 0;
}

core_initcall(leds_init);
