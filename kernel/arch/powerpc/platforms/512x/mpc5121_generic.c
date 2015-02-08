

#include <linux/kernel.h>
#include <linux/of_platform.h>

#include <asm/machdep.h>
#include <asm/ipic.h>
#include <asm/prom.h>
#include <asm/time.h>

#include "mpc512x.h"

static char *board[] __initdata = {
	"prt,prtlvt",
	NULL
};

static int __init mpc5121_generic_probe(void)
{
	unsigned long node = of_get_flat_dt_root();
	int i = 0;

	while (board[i]) {
		if (of_flat_dt_is_compatible(node, board[i]))
			break;
		i++;
	}

	return board[i] != NULL;
}

define_machine(mpc5121_generic) {
	.name			= "MPC5121 generic",
	.probe			= mpc5121_generic_probe,
	.init			= mpc512x_init,
	.init_IRQ		= mpc512x_init_IRQ,
	.get_irq		= ipic_get_irq,
	.calibrate_decr		= generic_calibrate_decr,
	.restart		= mpc512x_restart,
};
