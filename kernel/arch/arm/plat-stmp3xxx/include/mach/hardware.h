

#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H

#define IO_BASE			0xF0000000                 /* VA of IO  */
#define IO_SIZE			0x00100000                 /* How much? */
#define IO_START		0x80000000                 /* PA of IO  */

/* macro to get at IO space when running virtually */
#define IO_ADDRESS(x) (((x) & 0x000fffff) | IO_BASE)

#endif
