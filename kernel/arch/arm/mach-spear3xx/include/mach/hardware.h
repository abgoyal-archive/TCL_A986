

#ifndef __MACH_HARDWARE_H
#define __MACH_HARDWARE_H

/* Vitual to physical translation of statically mapped space */
#define IO_ADDRESS(x)		(x | 0xF0000000)

#endif /* __MACH_HARDWARE_H */
