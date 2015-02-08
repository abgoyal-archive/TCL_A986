

#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H

/* PCI IO info */
#define PCIO_BASE		IXP23XX_PCI_IO_VIRT
#define PCIBIOS_MIN_IO		0x00000000
#define PCIBIOS_MIN_MEM		0xe0000000

#include "ixp23xx.h"

#define pcibios_assign_all_busses()	0

#include "platform.h"

#include "ixdp2351.h"


#endif
