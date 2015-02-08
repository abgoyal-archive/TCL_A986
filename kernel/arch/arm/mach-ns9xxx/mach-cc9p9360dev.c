
#include <asm/mach/arch.h>
#include <asm/mach-types.h>

#include <mach/processor-ns9360.h>

#include "board-a9m9750dev.h"
#include "generic.h"

static void __init mach_cc9p9360dev_map_io(void)
{
	ns9360_map_io();
	board_a9m9750dev_map_io();
}

static void __init mach_cc9p9360dev_init_irq(void)
{
	ns9xxx_init_irq();
	board_a9m9750dev_init_irq();
}

static void __init mach_cc9p9360dev_init_machine(void)
{
	ns9xxx_init_machine();
	board_a9m9750dev_init_machine();
}

MACHINE_START(CC9P9360DEV, "Digi ConnectCore 9P 9360 on an A9M9750 Devboard")
	.map_io = mach_cc9p9360dev_map_io,
	.init_irq = mach_cc9p9360dev_init_irq,
	.init_machine = mach_cc9p9360dev_init_machine,
	.timer = &ns9360_timer,
	.boot_params = 0x100,
MACHINE_END
