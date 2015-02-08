
#ifndef __ASM_R4K_TYPES_H
#define __ASM_R4K_TYPES_H

#include <linux/compiler.h>

#ifdef CONFIG_SYNC_R4K

extern void synchronise_count_master(void);
extern void synchronise_count_slave(void);

#else

static inline void synchronise_count_master(void)
{
}

static inline void synchronise_count_slave(void)
{
}

#endif

#endif /* __ASM_R4K_TYPES_H */
