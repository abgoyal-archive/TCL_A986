

#ifndef __HARDWARE_H
#define __HARDWARE_H

#include <asm/types.h>

#define pcibios_assign_all_busses() 1
#define PCIBIOS_MIN_IO		0x00000000
#define PCIBIOS_MIN_MEM		0x00000000

#ifndef __ASSEMBLY__
void iop32x_init_irq(void);
#endif


#include "iop32x.h"

#include "glantank.h"
#include "iq80321.h"
#include "iq31244.h"
#include "n2100.h"


#endif
