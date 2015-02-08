
#ifndef __ASM_MACH_RM_MC146818RTC_H
#define __ASM_MACH_RM_MC146818RTC_H

#ifdef CONFIG_CPU_BIG_ENDIAN
#define mc146818_decode_year(year) ((year) < 70 ? (year) + 2000 : (year) + 1900)
#else
#define mc146818_decode_year(year) ((year) + 1980)
#endif

#include_next <mc146818rtc.h>

#endif /* __ASM_MACH_RM_MC146818RTC_H */
