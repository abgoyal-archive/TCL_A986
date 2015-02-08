

#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

/* physical offset of RAM */
#if defined(CONFIG_ARCH_QSD8X50) && defined(CONFIG_MSM_SOC_REV_A)
#define PHYS_OFFSET		UL(0x00000000)
#elif defined(CONFIG_ARCH_QSD8X50)
#define PHYS_OFFSET		UL(0x20000000)
#elif defined(CONFIG_ARCH_MSM7X30)
#define PHYS_OFFSET		UL(0x00200000)
#else
#define PHYS_OFFSET		UL(0x10000000)
#endif

#endif

