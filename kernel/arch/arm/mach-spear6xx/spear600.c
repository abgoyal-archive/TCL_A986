

#include <linux/ptrace.h>
#include <asm/irq.h>
#include <mach/generic.h>
#include <mach/spear.h>

/* Add spear600 specific devices here */

void __init spear600_init(void)
{
	/* call spear6xx family common init function */
	spear6xx_init();
}
