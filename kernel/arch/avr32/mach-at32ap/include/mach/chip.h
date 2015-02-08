
#ifndef __ASM_AVR32_ARCH_CHIP_H__
#define __ASM_AVR32_ARCH_CHIP_H__

#if defined(CONFIG_CPU_AT32AP700X)
# include <mach/at32ap700x.h>
#else
# error Unknown chip type selected
#endif

#endif /* __ASM_AVR32_ARCH_CHIP_H__ */
