

#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H

#ifndef __ASSEMBLY__

extern unsigned int s3c2410_modify_misccr(unsigned int clr, unsigned int chg);

#ifdef CONFIG_CPU_S3C2440

extern int s3c2440_set_dsc(unsigned int pin, unsigned int value);

#endif /* CONFIG_CPU_S3C2440 */

#ifdef CONFIG_CPU_S3C2412

extern int s3c2412_gpio_set_sleepcfg(unsigned int pin, unsigned int state);

#endif /* CONFIG_CPU_S3C2412 */

#endif /* __ASSEMBLY__ */

#include <asm/sizes.h>
#include <mach/map.h>

/* machine specific hardware definitions should go after this */

/* currently here until moved into config (todo) */
#define CONFIG_NO_MULTIWORD_IO

#endif /* __ASM_ARCH_HARDWARE_H */
