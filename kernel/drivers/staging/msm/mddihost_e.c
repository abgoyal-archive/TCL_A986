

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>

#include "msm_fb.h"
#include "mddihost.h"
#include "mddihosti.h"

#include <linux/clk.h>
#include <mach/clk.h>

extern struct semaphore mddi_host_mutex;
static boolean mddi_host_ext_powered = FALSE;

void mddi_host_start_ext_display(void)
{
	down(&mddi_host_mutex);

	if (!mddi_host_ext_powered) {
		mddi_host_init(MDDI_HOST_EXT);

		mddi_host_ext_powered = TRUE;
	}

	up(&mddi_host_mutex);
}

void mddi_host_stop_ext_display(void)
{
	down(&mddi_host_mutex);

	if (mddi_host_ext_powered) {
		mddi_host_powerdown(MDDI_HOST_EXT);

		mddi_host_ext_powered = FALSE;
	}

	up(&mddi_host_mutex);
}
