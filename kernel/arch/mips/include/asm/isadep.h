

#ifndef __ASM_ISADEP_H
#define __ASM_ISADEP_H

#if defined(CONFIG_CPU_R3000) || defined(CONFIG_CPU_TX39XX)

#define KU_MASK 0x08
#define	KU_USER 0x08
#define KU_KERN 0x00

#else
#define KU_MASK 0x18
#define	KU_USER 0x10
#define KU_KERN 0x00

#endif

#endif /* __ASM_ISADEP_H */
