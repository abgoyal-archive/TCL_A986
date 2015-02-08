

#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/param.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/device.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/rtc.h>
#include <asm/machvec.h>
#include <mach/sysasic.h>

static void __init dreamcast_setup(char **cmdline_p)
{
	board_time_init = aica_time_init;
}

static struct sh_machine_vector mv_dreamcast __initmv = {
	.mv_name		= "Sega Dreamcast",
	.mv_setup		= dreamcast_setup,
	.mv_irq_demux		= systemasic_irq_demux,
	.mv_init_irq		= systemasic_irq_init,
};
