

#include <linux/init.h>
#include <linux/platform_device.h>
#include <asm/io.h>
#include <asm/machvec.h>


static struct sh_machine_vector mv_se __initmv = {
	.mv_name		= "SolutionEngine",
	.mv_nr_irqs		= 108,
};
