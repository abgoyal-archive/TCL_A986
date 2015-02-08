

#ifndef __MACH_HARDWARE_H
#define __MACH_HARDWARE_H

#include <asm/sizes.h>

/* macro to get at IO space when running virtually */
#define PCIBIOS_MIN_IO		0x00000000
#define PCIBIOS_MIN_MEM		0x00000000
#define pcibios_assign_all_busses()	1

#endif
