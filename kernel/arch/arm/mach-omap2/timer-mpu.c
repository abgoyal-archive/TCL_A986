
#include <linux/init.h>
#include <linux/smp.h>
#include <linux/clockchips.h>
#include <asm/irq.h>
#include <asm/smp_twd.h>
#include <asm/localtimer.h>

void __cpuinit local_timer_setup(struct clock_event_device *evt)
{
	evt->irq = OMAP44XX_IRQ_LOCALTIMER;
	twd_timer_setup(evt);
}

