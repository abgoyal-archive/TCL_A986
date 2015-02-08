

#ifndef __ASM_VDSO_H
#define __ASM_VDSO_H

#include <linux/types.h>


#ifdef CONFIG_32BIT
struct mips_vdso {
	u32 signal_trampoline[2];
	u32 rt_signal_trampoline[2];
};
#else  /* !CONFIG_32BIT */
struct mips_vdso {
	u32 o32_signal_trampoline[2];
	u32 o32_rt_signal_trampoline[2];
	u32 rt_signal_trampoline[2];
	u32 n32_rt_signal_trampoline[2];
};
#endif /* CONFIG_32BIT */

#endif /* __ASM_VDSO_H */
