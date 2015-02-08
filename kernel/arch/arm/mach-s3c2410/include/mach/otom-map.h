

/* needs arch/map.h including with this */


#ifndef __ASM_ARCH_OTOMMAP_H
#define __ASM_ARCH_OTOMMAP_H

#define OTOM_PA_CS8900A_BASE       (S3C2410_CS3 + 0x01000000)	/* nGCS3 +0x01000000 */
#define OTOM_VA_CS8900A_BASE       S3C2410_ADDR(0x04000000)		/* 0xF4000000 */

/* physical offset addresses for the peripherals */

#define OTOM_PA_FLASH0_BASE        (S3C2410_CS0)				/* Bank 0 */

#endif /* __ASM_ARCH_OTOMMAP_H */
