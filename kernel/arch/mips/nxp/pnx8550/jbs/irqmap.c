

#include <linux/init.h>
#include <int.h>

char pnx8550_irq_tab[][5] __initdata = {
	[8]	= { -1, PNX8550_INT_PCI_INTA, 0xff, 0xff, 0xff},
	[9]	= { -1, PNX8550_INT_PCI_INTA, 0xff, 0xff, 0xff},
	[17]	= { -1, PNX8550_INT_PCI_INTA, 0xff, 0xff, 0xff},
};
