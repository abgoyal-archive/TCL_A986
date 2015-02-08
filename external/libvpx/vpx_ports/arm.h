


#ifndef VPX_PORTS_ARM_H
#define VPX_PORTS_ARM_H
#include <stdlib.h>
#include "config.h"

/*ARMv5TE "Enhanced DSP" instructions.*/
#define HAS_EDSP  0x01
/*ARMv6 "Parallel" or "Media" instructions.*/
#define HAS_MEDIA 0x02
/*ARMv7 optional NEON instructions.*/
#define HAS_NEON  0x04

int arm_cpu_caps(void);

#endif

