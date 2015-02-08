
#ifndef IP2_H
#define IP2_H

#include "ip2types.h"
#include "i2cmd.h"

/*************/
/* Constants */
/*************/

/* Device major numbers - since version 2.0.26. */
#define IP2_TTY_MAJOR      71
#define IP2_CALLOUT_MAJOR  72
#define IP2_IPL_MAJOR      73


 /* this structure is zeroed out because the suggested method is to configure
  * the driver as a module, set up the parameters with an options line in
  * /etc/modprobe.conf and load with modprobe or kmod, the kernel
  * module loader
  */

 /* This structure is NOW always initialized when the driver is initialized.
  * Compiled in defaults MUST be added to the io and irq arrays in
  * ip2.c.  Those values are configurable from insmod parameters in the
  * case of modules or from command line parameters (ip2=io,irq) when
  * compiled in.
  */

static ip2config_t ip2config =
{
	{0,0,0,0},		// irqs
	{				// Addresses
	/* Do NOT set compile time defaults HERE!  Use the arrays in
		ip2.c!  These WILL be overwritten!  =mhw= */
		0x0000,		// Board 0, ttyF0   - ttyF63
		0x0000,		// Board 1, ttyF64  - ttyF127
		0x0000,		// Board 2, ttyF128 - ttyF191
		0x0000		// Board 3, ttyF192 - ttyF255
	}
};

#endif
