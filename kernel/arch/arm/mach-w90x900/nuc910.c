

#include <linux/platform_device.h>
#include <asm/mach/map.h>
#include <mach/hardware.h>
#include "cpu.h"
#include "clock.h"

/* define specific CPU platform device */

static struct platform_device *nuc910_dev[] __initdata = {
	&nuc900_device_ts,
	&nuc900_device_rtc,
};

/* define specific CPU platform io map */

static struct map_desc nuc910evb_iodesc[] __initdata = {
	IODESC_ENT(USBEHCIHOST),
	IODESC_ENT(USBOHCIHOST),
	IODESC_ENT(KPI),
	IODESC_ENT(USBDEV),
	IODESC_ENT(ADC),
};

/*Init NUC910 evb io*/

void __init nuc910_map_io(void)
{
	nuc900_map_io(nuc910evb_iodesc, ARRAY_SIZE(nuc910evb_iodesc));
}

/*Init NUC910 clock*/

void __init nuc910_init_clocks(void)
{
	nuc900_init_clocks();
}

/*Init NUC910 board info*/

void __init nuc910_board_init(void)
{
	nuc900_board_init(nuc910_dev, ARRAY_SIZE(nuc910_dev));
}
