
#include <linux/init.h>
#include <linux/smp.h>
#include <linux/clockchips.h>

#include <asm/smp_twd.h>
#include <asm/localtimer.h>
#include <mach/irqs.h>

void __cpuinit local_timer_setup(struct clock_event_device *evt)
{
	evt->irq = IRQ_LOCALTIMER;
	twd_timer_setup(evt);
}
