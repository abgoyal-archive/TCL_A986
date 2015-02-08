

#ifndef __ASM_ARCH_HARDWARE_H__
#define __ASM_ARCH_HARDWARE_H__

#define PCIBIOS_MIN_IO          0x00000000
#define PCIBIOS_MIN_MEM         0x00000000

#include "ixp2000-regs.h"	/* Chipset Registers */

#define pcibios_assign_all_busses() 0

#include "platform.h"

#include "enp2611.h"		/* ENP-2611 */
#include "ixdp2x00.h"		/* IXDP2400/2800 */
#include "ixdp2x01.h"		/* IXDP2401/2801 */

#endif  /* _ASM_ARCH_HARDWARE_H__ */
