
#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H

#include <asm/sizes.h>
#include <mach/platform.h>

/* Start of virtual addresses for IO devices */
#define IO_BASE         0xF0000000

/* This macro relies on fact that for all HW i/o addresses bits 20-23 are 0 */
#define IO_ADDRESS(x)  (((((x) & 0xff000000) >> 4) | ((x) & 0xfffff)) | IO_BASE)

#endif
