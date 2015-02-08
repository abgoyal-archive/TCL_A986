

#include <linux/platform_device.h>
#include <asm/mach/map.h>
#include <mach/hardware.h>

#include "cpu.h"

/* define specific CPU platform device */

static struct platform_device *nuc950_dev[] __initdata = {
	&nuc900_device_kpi,
	&nuc900_device_fmi,
#ifdef CONFIG_FB_NUC900
	&nuc900_device_lcd,
#endif
};

/* define specific CPU platform io map */

static struct map_desc nuc950evb_iodesc[] __initdata = {
};

/*Init NUC950 evb io*/

void __init nuc950_map_io(void)
{
	nuc900_map_io(nuc950evb_iodesc, ARRAY_SIZE(nuc950evb_iodesc));
}

/*Init NUC950 clock*/

void __init nuc950_init_clocks(void)
{
	nuc900_init_clocks();
}

/*Init NUC950 board info*/

void __init nuc950_board_init(void)
{
	nuc900_board_init(nuc950_dev, ARRAY_SIZE(nuc950_dev));
}
