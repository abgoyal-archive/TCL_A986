
#ifndef __ARM_MTD_XIP_H__
#define __ARM_MTD_XIP_H__

#include <asm/hardware.h>
#include <asm/arch/mtd-xip.h>

/* fill instruction prefetch */
#define xip_iprefetch() 	do { asm volatile (".rep 8; nop; .endr"); } while (0)

#endif /* __ARM_MTD_XIP_H__ */
