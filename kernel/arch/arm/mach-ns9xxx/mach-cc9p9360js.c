
#include <asm/mach/arch.h>
#include <asm/mach-types.h>

#include <mach/processor-ns9360.h>

#include "board-jscc9p9360.h"
#include "generic.h"

static void __init mach_cc9p9360js_init_machine(void)
{
	ns9xxx_init_machine();
	board_jscc9p9360_init_machine();
}

MACHINE_START(CC9P9360JS, "Digi ConnectCore 9P 9360 on an JSCC9P9360 Devboard")
	.map_io = ns9360_map_io,
	.init_irq = ns9xxx_init_irq,
	.init_machine = mach_cc9p9360js_init_machine,
	.timer = &ns9360_timer,
	.boot_params = 0x100,
MACHINE_END
