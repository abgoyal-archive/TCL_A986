

#undef DEBUG
#include <asm/time.h>
#include <asm/prom.h>
#include <asm/machdep.h>
#include <asm/mpc52xx.h>

static void __init mpc5200_simple_setup_arch(void)
{
	if (ppc_md.progress)
		ppc_md.progress("mpc5200_simple_setup_arch()", 0);

	/* Map important registers from the internal memory map */
	mpc52xx_map_common_devices();

	/* Some mpc5200 & mpc5200b related configuration */
	mpc5200_setup_xlb_arbiter();

	mpc52xx_setup_pci();
}

/* list of the supported boards */
static char *board[] __initdata = {
	"intercontrol,digsy-mtc",
	"manroland,mucmc52",
	"manroland,uc101",
	"phytec,pcm030",
	"phytec,pcm032",
	"promess,motionpro",
	"schindler,cm5200",
	"tqc,tqm5200",
	NULL
};

static int __init mpc5200_simple_probe(void)
{
	unsigned long node = of_get_flat_dt_root();
	int i = 0;

	while (board[i]) {
		if (of_flat_dt_is_compatible(node, board[i]))
			break;
		i++;
	}
	
	return (board[i] != NULL);
}

define_machine(mpc5200_simple_platform) {
	.name		= "mpc5200-simple-platform",
	.probe		= mpc5200_simple_probe,
	.setup_arch	= mpc5200_simple_setup_arch,
	.init		= mpc52xx_declare_of_platform_devices,
	.init_IRQ	= mpc52xx_init_irq,
	.get_irq	= mpc52xx_get_irq,
	.restart	= mpc52xx_restart,
	.calibrate_decr	= generic_calibrate_decr,
};
