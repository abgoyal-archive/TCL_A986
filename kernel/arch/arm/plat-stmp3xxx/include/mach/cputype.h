

#ifndef __ASM_PLAT_CPU_H
#define __ASM_PLAT_CPU_H

#ifdef CONFIG_ARCH_STMP37XX
#define cpu_is_stmp37xx()	(1)
#else
#define cpu_is_stmp37xx()	(0)
#endif

#ifdef CONFIG_ARCH_STMP378X
#define cpu_is_stmp378x()	(1)
#else
#define cpu_is_stmp378x()	(0)
#endif

#endif /* __ASM_PLAT_CPU_H */
